"""Tune SMC per joint; only an explicit test opens ROS/CAN."""

import argparse
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace

from .algorithm_tuner import TuningSession
from .smc_joint_trial import JOINTS, JOINT_NAMES, RosSmcJointTrial, checked_trial


HELP = """SMC từng khớp: elbow/id2 -> shoulder/id1 -> base/id0.
Hai khớp còn lại giữ position. tau = tau_ff(URDF) + tau_fb(SMC).
  show                    Bộ số của khớp đang chọn, YAML và giới hạn
  set LAMBDA KS KR PHI     Chỉnh riêng khớp đang chọn, chưa ghi YAML
  lambda / ks / kr / phi VALUE
  slew VALUE              Giới hạn tốc độ đổi feedback torque [Nm/s]
  alpha VALUE             Hệ số feedforward riêng base; chỉ chỉnh khi chọn base
  test DEG [MOVE HOLD]    Đi thêm DEG độ từ HOME, giữ, về HOME, giữ (mặc định 6/3 s)
  save                    Lưu bộ số vào YAML trên máy
  reset                   Đọc lại YAML, bỏ các thay đổi chưa save
  next                    Đổi khớp; phải restart launch với torque_joint tương ứng
  help / quit
Ctrl-C trong test: trả về position trước khi đóng node torque và ghi log.
Đóng cửa sổ đồ thị để quay lại prompt. save trước next/quit nếu muốn giữ gain.
"""
FIELDS = ('lambda_gain', 'ks', 'kr', 'phi')


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Giá trị phải hữu hạn và > 0')
    return value


def validate_joint_profile(params, selected):
    if float(params['control_hz']) != 100.0:
        raise ValueError('Tune phần cứng dùng control_hz=100')
    if positive(params['tau_scale']) > 1:
        raise ValueError('tau_scale phải trong (0, 1]')
    positive(params['base_ff_alpha'])
    slew = params['torque_slew_rate']
    if len(slew) != 3:
        raise ValueError('torque_slew_rate cần ba giá trị: base shoulder elbow')
    for value in slew:
        positive(value)
    return ideal_radii(params, selected)


def ideal_radii(params, selected):
    from gim_arm_controller_smc.controller import SmcController
    controller = SmcController(
        SimpleNamespace(nq=3, joint_names=JOINT_NAMES, tau_max=[5, 40, 5]),
        **{name: params[name] for name in FIELDS}, control_hz=params['control_hz'],
        controlled_joint=JOINTS[selected])
    radii = controller.sampled_spectral_radius()
    if radii[selected] >= 1:
        raise ValueError('Bộ số không ổn định trong xấp xỉ ZOH 100 Hz; '
                         'giảm lambda/ks/kr hoặc tăng phi')
    return radii


class SmcJointTerminal:
    def __init__(self, params_file, joint='elbow', log_dir='results/smc_joint_trials',
                 show_plot=True, dry_run=False, interface='can0', prestart_timeout=6.0):
        self.session = TuningSession('smc', params_file)
        self.selected = JOINTS.index(joint)
        validate_joint_profile(self.parameters, self.selected)
        self.dry_run, self.interface = dry_run, interface
        self.trial = RosSmcJointTrial(log_dir, show_plot, prestart_timeout)

    @property
    def parameters(self):
        return self.session.params

    def replace(self, params):
        document = deepcopy(self.session.document)
        document['smc_controller']['ros__parameters'] = params
        self.session.validate(document)
        validate_joint_profile(params, self.selected)
        self.session.document = document

    def set_gains(self, *values):
        params = deepcopy(self.parameters)
        for name, value in zip(FIELDS, values):
            vector = list(params[name])
            vector[self.selected] = float(value)
            params[name] = self.session.parse_value(name, vector)
        self.replace(params)

    def show(self):
        params, index = self.parameters, self.selected
        print(f'Khớp {JOINTS[index]}/id{index}: ' + ', '.join(
            f'{name}={params[name][index]:g}' for name in FIELDS))
        rho = validate_joint_profile(params, index)[index]
        print(f'100 Hz; tau_scale={params["tau_scale"]:g}; '
              f'feedback slew={params["torque_slew_rate"][index]:g} Nm/s; '
              f'base alpha={params["base_ff_alpha"]:g}; rho_ZOH_ideal={rho:.4f}')
        print(f'YAML lưu bằng save: {self.session.params_file}')

    def stage(self):
        self.show()
        print('Đặt/đỡ tay đúng zero rồi launch phần cứng với:')
        print('ros2 launch gim_control origin_gim_arm_control.launch.py '
              f'can_interface:={self.interface} set_zero_on_startup:=true '
              f'zero_method:=software torque_joint:={JOINTS[self.selected]}')
        print('Chờ position active, chạy HOME về 0 trước test; đọc README_TUNE_SMC.md.')

    def execute(self, line):
        import shlex
        words = shlex.split(line)
        if not words:
            return True
        command, *args = words
        if command in ('quit', 'exit') and not args:
            return False
        if command == 'help' and not args:
            print(HELP)
        elif command == 'show' and not args:
            self.show()
        elif command == 'set' and len(args) == 4:
            self.set_gains(*args)
            self.show()
        elif command in ('lambda', 'lambda_gain', 'ks', 'kr', 'phi') and len(args) == 1:
            name = 'lambda_gain' if command == 'lambda' else command
            values = [self.parameters[field][self.selected] for field in FIELDS]
            values[FIELDS.index(name)] = args[0]
            self.set_gains(*values)
            self.show()
        elif command in ('slew', 'alpha') and len(args) == 1:
            params = deepcopy(self.parameters)
            if command == 'alpha':
                if self.selected != 0:
                    raise ValueError('alpha chỉ chỉnh khi đang chọn base')
                params['base_ff_alpha'] = positive(args[0])
            else:
                params['torque_slew_rate'][self.selected] = positive(args[0])
            self.replace(params)
            self.show()
        elif command == 'reset' and not args:
            params = deepcopy(self.session.document)
            try:
                self.session.reset()
                validate_joint_profile(self.parameters, self.selected)
            except (ValueError, KeyError):
                self.session.document = params
                raise
            self.show()
        elif command == 'next' and not args:
            if self.selected > 0:
                validate_joint_profile(self.parameters, self.selected - 1)
                self.selected -= 1
                self.stage()
            else:
                print('Đã hoàn tất elbow -> shoulder -> base.')
        elif command == 'save' and not args:
            if self.dry_run:
                print('DRY RUN: không ghi YAML.')
            else:
                self.session.save()
        elif command == 'test' and 1 <= len(args) <= 3:
            values = checked_trial(args[0], args[1] if len(args) > 1 else 6,
                                   args[2] if len(args) > 2 else 3)
            validate_joint_profile(self.parameters, self.selected)
            if self.dry_run:
                print(f'DRY RUN: {JOINTS[self.selected]}, {values}; không mở ROS/CAN.')
            else:
                self.trial.run(self.selected, *values, deepcopy(self.parameters))
        else:
            raise ValueError('Sai lệnh; gõ help để xem.')
        return True

    def run(self):
        print(HELP)
        self.stage()
        while True:
            try:
                if not self.execute(input(f'smc[{JOINTS[self.selected]}/id{self.selected}]> ')):
                    return
            except EOFError:
                return
            except KeyboardInterrupt:
                print('\nĐã ngắt; chờ thông báo trả về position trước khi tiếp tục.')
            except (ValueError, RuntimeError, OSError) as error:
                print(f'Lỗi: {error}')


def main(root, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', choices=JOINTS, default='elbow')
    parser.add_argument('--params-file', type=Path, default=Path(root) /
                        'src/gim_arm_controller_smc/config/smc_joint_tuning.yaml')
    parser.add_argument('--log-dir', default='results/smc_joint_trials')
    parser.add_argument('--interface', '--can', default='can0')
    parser.add_argument('--prestart-timeout', type=positive, default=6.0)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        SmcJointTerminal(args.params_file, args.joint, args.log_dir, not args.no_show,
                         args.dry_run, args.interface, args.prestart_timeout).run()
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
