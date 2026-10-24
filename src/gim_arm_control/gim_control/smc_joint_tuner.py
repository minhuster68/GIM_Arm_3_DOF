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
STA_FIELDS = ('lambda_gain', 'k1', 'k2')


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Giá trị phải hữu hạn và > 0')
    return value


def validate_joint_profile(params, selected):
    if float(params['control_hz']) != 100.0:
        raise ValueError('Tune phần cứng dùng control_hz=100')
    if params.get('use_sim_time', False):
        raise ValueError('Tune phần cứng cần use_sim_time=false')
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
    if params.get('smc_variant', 'saturation') == 'super_twisting':
        from gim_control.controller_api import vector_parameter
        from gim_arm_controller_smc.velocity_kalman import VelocityKalman2
        node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=params[name]))
        for name in (*STA_FIELDS, 'z_limit'):
            vector_parameter(node, name, 3, positive=True)
        kalman = VelocityKalman2(
            3, params.get('velocity_kalman_enabled'), params.get('velocity_kalman_jerk_psd'),
            params.get('velocity_kalman_measurement_variance'))
        if any(enabled and hz > 0 for enabled, hz in zip(
                kalman.enabled, params.get('velocity_lpf_hz', [0., 0., 0.]))):
            raise ValueError('Không bật LPF và Kalman cùng một khớp')
        # The saturation boundary-layer ZOH test does not apply to STA.
        return None
    if params.get('smc_variant', 'saturation') != 'saturation':
        raise ValueError('smc_variant phải là saturation hoặc super_twisting')
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
    def __init__(self, params_file, joint='elbow', log_dir='results_new/smc/joint_trials',
                 show_plot=True, dry_run=False, interface='can0', prestart_timeout=6.0):
        self.session = TuningSession('smc', params_file)
        self.selected = JOINTS.index(joint)
        validate_joint_profile(self.parameters, self.selected)
        self.dry_run, self.interface = dry_run, interface
        self.trial = RosSmcJointTrial(
            log_dir, show_plot, prestart_timeout, can_interface=interface)

    @property
    def parameters(self):
        return self.session.params

    @property
    def is_sta(self):
        return self.parameters.get('smc_variant', 'saturation') == 'super_twisting'

    @property
    def fields(self):
        return STA_FIELDS if self.is_sta else FIELDS

    def help_text(self):
        if not self.is_sta:
            return HELP
        return HELP.replace('SMC từng khớp:', 'SMC super-twisting từng khớp:').replace(
            'set LAMBDA KS KR PHI', 'set LAMBDA K1 K2').replace(
            'lambda / ks / kr / phi VALUE',
            'lambda / k1 / k2 VALUE\n  zlimit VALUE            Giới hạn trạng thái z [rad/s²]')

    def replace(self, params):
        document = deepcopy(self.session.document)
        document['smc_controller']['ros__parameters'] = params
        self.session.validate(document)
        validate_joint_profile(params, self.selected)
        self.session.document = document

    def set_gains(self, *values):
        if len(values) != len(self.fields):
            raise ValueError('set ' + ' '.join(
                'LAMBDA' if name == 'lambda_gain' else name.upper() for name in self.fields))
        params = deepcopy(self.parameters)
        for name, value in zip(self.fields, values):
            vector = list(params[name])
            vector[self.selected] = float(value)
            params[name] = self.session.parse_value(name, vector)
        self.replace(params)

    def show(self):
        params, index = self.parameters, self.selected
        variant = 'super-twisting' if self.is_sta else 'saturation'
        print(f'SMC {variant} | Khớp {JOINTS[index]}/id{index}: ' + ', '.join(
            f'{name}={params[name][index]:g}' for name in self.fields))
        radii = validate_joint_profile(params, index)
        stability = ('ZOH lớp biên không áp dụng cho STA' if radii is None else
                     f'rho_ZOH_ideal={radii[index]:.4f}')
        print(f'100 Hz; tau_scale={params["tau_scale"]:g}; '
              f'feedback slew={params["torque_slew_rate"][index]:g} Nm/s; '
              f'base alpha={params["base_ff_alpha"]:g}; {stability}')
        if self.is_sta:
            print(f'z_limit={params["z_limit"][index]:g} rad/s²; '
                  'z reset về 0 khi bắt đầu lượt test mới')
            if params.get('velocity_kalman_enabled', [False] * 3)[index]:
                print('Kalman [v, a]: '
                      f'jerk PSD={params.get("velocity_kalman_jerk_psd", [1.] * 3)[index]:g}; '
                      f'R={params.get("velocity_kalman_measurement_variance", [.0004] * 3)[index]:g}')
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
            print(self.help_text())
        elif command == 'show' and not args:
            self.show()
        elif command == 'set':
            self.set_gains(*args)
            self.show()
        elif (command == 'lambda' or command in self.fields) and len(args) == 1:
            name = 'lambda_gain' if command == 'lambda' else command
            values = [self.parameters[field][self.selected] for field in self.fields]
            values[self.fields.index(name)] = args[0]
            self.set_gains(*values)
            self.show()
        elif (command in ('slew', 'alpha') or
              (self.is_sta and command in ('zlimit', 'z_limit'))) and len(args) == 1:
            params = deepcopy(self.parameters)
            if command == 'alpha':
                if self.selected != 0:
                    raise ValueError('alpha chỉ chỉnh khi đang chọn base')
                params['base_ff_alpha'] = positive(args[0])
            else:
                name = 'z_limit' if command in ('zlimit', 'z_limit') else 'torque_slew_rate'
                params[name][self.selected] = positive(args[0])
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
        print(self.help_text())
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
                        'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml')
    parser.add_argument('--log-dir', default='results_new/smc/joint_trials')
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
