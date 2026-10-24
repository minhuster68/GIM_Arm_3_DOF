"""Tune one MPC torque joint at a time; only explicit test connects to ROS/CAN."""

import argparse
from copy import deepcopy
import math
from pathlib import Path
import shlex

from .algorithm_tuner import TuningSession
from .mpc_joint_trial import JOINTS, RosMpcJointTrial, checked_trial


HELP = """MPC từng khớp: elbow (3) -> shoulder (2) -> base (1).
tau = tau_ff(URDF) + tau_fb(QP). Q phạt tích phân, vị trí và vận tốc;
đánh giá tracking theo vị trí. Hai khớp còn lại giữ position.
  show                    Xem trọng số thực trong Q/R/Rd và YAML
  set QE QI QV R [RD]      Chỉnh trọng số riêng khớp đang chọn; bỏ RD để giữ cũ
  qe / qi / qv / r / rd VALUE
  slew VALUE              Tốc độ đổi feedback torque [Nm/s]
  alpha VALUE             Nhân tau_ff riêng base; chỉ chỉnh khi chọn base
  np N / nc N             Prediction/control horizon (số bước 10 ms, dùng chung)
  test DEG [MOVE HOLD]    Đi thêm DEG độ, giữ, về HOME; mặc định 6 / 3 s
  save                    Lưu YAML trên máy; không ghi flash motor
  next                    Chuyển elbow -> shoulder -> base
  help / quit
QE/QI/QV/R > 0; RD >= 0. Nhập trọng số thực, terminal tự đổi về YAML.
RD phạt thay đổi feedback torque; slew là ràng buộc cứng của QP.
Ctrl-C trong test: trả về position, lưu log và mở đồ thị.
"""
COSTS = ('qe', 'qi', 'qv', 'r', 'rd')
FIELDS = ('q_position', 'q_integral', 'q_velocity', 'r_input', 'r_rate')
SCALES = ('output_scale_position', 'output_scale_integral',
          'output_scale_velocity', 'input_scale', 'input_scale')


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Giá trị phải hữu hạn và > 0')
    return value


def validate_joint_profile(params):
    if float(params['control_hz']) != 100.0:
        raise ValueError('Tune phần cứng dùng control_hz=100')
    if params.get('weight_convention') != 'matlab_squared':
        raise ValueError('Terminal cần weight_convention=matlab_squared')
    if params.get('feedback_bounds_mode') != 'trajectory_global' or params.get('precompute_prediction') is not True:
        raise ValueError('Cần trajectory_global và precompute_prediction=true')
    if positive(params['tau_scale']) > 1:
        raise ValueError('tau_scale phải trong (0, 1]')
    if params.get('feedback_model_mode', 'coupled') not in ('coupled', 'independent'):
        raise ValueError('feedback_model_mode phải là coupled hoặc independent')
    positive(params.get('base_ff_alpha', 1.0))
    penalty = positive(params['torque_penalty_scale'])
    for field, scale in zip(FIELDS, SCALES):
        if len(params[field]) != 3 or len(params[scale]) != 3:
            raise ValueError(f'{field}/{scale} cần ba giá trị')
        for weight, divisor in zip(params[field], params[scale]):
            value = float(weight) / positive(divisor)
            cost = value**2 * (penalty if field.startswith('r_') else 1)
            if not math.isfinite(cost) or cost < 0 or (field in ('q_position', 'r_input') and cost == 0):
                raise ValueError('Trọng số MPC ngoài phạm vi số hữu hạn')


class MpcJointTerminal:
    def __init__(self, params_file, joint='elbow', log_dir='results_new/mpc/joint_trials',
                 show_plot=True, dry_run=False, interface='can0', prestart_timeout=6.0):
        self.session = TuningSession('mpc', params_file)
        validate_joint_profile(self.parameters)
        self.parameters.setdefault('base_ff_alpha', 1.0)
        self.parameters.setdefault('feedback_model_mode', 'coupled')
        self.selected = JOINTS.index(joint)
        self.dry_run, self.interface = dry_run, interface
        self.trial = RosMpcJointTrial(log_dir, show_plot, prestart_timeout)

    @property
    def parameters(self):
        return self.session.params

    def costs(self):
        p, i = self.parameters, self.selected
        return tuple((p[field][i] / p[scale][i])**2 *
                     (p['torque_penalty_scale'] if field.startswith('r_') else 1)
                     for field, scale in zip(FIELDS, SCALES))

    def replace(self, params):
        candidate = deepcopy(self.session.document)
        candidate['mpc_controller']['ros__parameters'] = params
        self.session.validate(candidate)
        validate_joint_profile(params)
        self.session.document = candidate

    def set_costs(self, qe, qi, qv, r, rd=None):
        values = [positive(v) for v in (qe, qi, qv, r)]
        rd = self.costs()[-1] if rd is None else float(rd)
        if not math.isfinite(rd) or rd < 0:
            raise ValueError('RD phải hữu hạn và >= 0')
        values.append(rd)
        p = deepcopy(self.parameters)
        for cost, field, scale in zip(values, FIELDS, SCALES):
            penalty = p['torque_penalty_scale'] if field.startswith('r_') else 1
            p[field][self.selected] = p[scale][self.selected] * math.sqrt(cost / penalty)
        self.replace(p)

    def show(self):
        print(f'Khâu {self.selected + 1}/{JOINTS[self.selected]} | ' +
              ', '.join(f'{name.upper()}={value:g}' for name, value in zip(COSTS, self.costs())))
        p = self.parameters
        print(f'feedback_model_mode={p["feedback_model_mode"]}')
        print(f'alpha base={p["base_ff_alpha"]:g}, tau_scale={p["tau_scale"]:g}, '
              f'slew={p["torque_slew_rate"][self.selected]:g} Nm/s, '
              f'Np={p["prediction_horizon"]}, Nc={p["control_horizon"]}')
        print(f'YAML lưu bằng save: {self.session.params_file}')

    def stage(self):
        self.show()
        print('Đặt/đỡ tay đúng zero trước startup, rồi launch:')
        print('ros2 launch gim_control origin_gim_arm_control.launch.py '
              f'can_interface:={self.interface} set_zero_on_startup:=true '
              f'zero_method:=software torque_joint:={JOINTS[self.selected]}')
        print('Đổi khớp cần khởi động lại launch; cùng khớp giữ launch chạy.')

    def execute(self, line):
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
        elif command == 'set' and len(args) in (4, 5):
            self.set_costs(*args)
            self.show()
        elif command in COSTS and len(args) == 1:
            costs = list(self.costs())
            costs[COSTS.index(command)] = args[0]
            self.set_costs(*costs)
            self.show()
        elif command in ('slew', 'alpha', 'np', 'nc') and len(args) == 1:
            p = deepcopy(self.parameters)
            if command == 'alpha':
                if self.selected != 0:
                    raise ValueError('alpha chỉ chỉnh khi đang chọn base')
                p['base_ff_alpha'] = positive(args[0])
            elif command == 'slew':
                p['torque_slew_rate'][self.selected] = positive(args[0])
            else:
                key = 'prediction_horizon' if command == 'np' else 'control_horizon'
                p[key] = self.session.parse_value(key, args)
            self.replace(p)
            self.show()
        elif command == 'next' and not args:
            if self.selected > 0:
                self.selected -= 1
                self.stage()
            else:
                print('Đã hoàn tất thứ tự 3 -> 2 -> 1.')
        elif command == 'save' and not args:
            if self.dry_run:
                print('DRY RUN: không ghi YAML.')
            else:
                self.session.save()
        elif command == 'test' and 1 <= len(args) <= 3:
            values = checked_trial(args[0], args[1] if len(args) > 1 else 6,
                                   args[2] if len(args) > 2 else 3)
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
                if not self.execute(input(f'mpc[khâu {self.selected + 1}/{JOINTS[self.selected]}]> ')):
                    return
            except EOFError:
                return
            except KeyboardInterrupt:
                print('\nĐã ngắt; kiểm tra thông báo trả về position trước khi tiếp tục.')
            except (ValueError, RuntimeError, OSError) as error:
                print(f'Lỗi: {error}')


def main(root, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', choices=JOINTS, default='elbow')
    parser.add_argument('--params-file', type=Path, default=Path(root) /
                        'src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml')
    parser.add_argument('--log-dir', default='results_new/mpc/joint_trials')
    parser.add_argument('--interface', '--can', default='can0')
    parser.add_argument('--prestart-timeout', type=positive, default=6.0)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        MpcJointTerminal(args.params_file, args.joint, args.log_dir, not args.no_show,
                         args.dry_run, args.interface, args.prestart_timeout).run()
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
