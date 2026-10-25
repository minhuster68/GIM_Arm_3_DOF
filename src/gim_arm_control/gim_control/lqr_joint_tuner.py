"""Tune selected-joint LQR Q/R costs; only explicit test uses ROS/CAN."""

import argparse
from copy import deepcopy
import math
from pathlib import Path
import shlex

from .algorithm_tuner import TuningSession
from .lqr_joint_trial import JOINTS, RosLqrJointTrial, checked_trial


HELP = """LQR từng khớp: elbow (3) -> shoulder (2) -> base (1).
Luật: tau = tau_ff(URDF) - K @ [integral(e), e, qd-qd_ref], e=q-q_ref.
Q gồm sai số tích phân, vị trí và vận tốc; đánh giá tracking theo vị trí.
  show                  Xem QE, QI, QV, R và YAML
  set QE QI QV R        Chỉnh bốn trọng số của riêng khớp đang chọn
  set QE QI R           Cú pháp cũ: giữ nguyên QV hiện tại
  qe VALUE / qi VALUE / qv VALUE / r VALUE
  max_e VALUE / max_int_e VALUE / max_de VALUE / max_tau VALUE
  slew VALUE            Giới hạn tốc độ đổi torque riêng khớp [Nm/s], > 0
  alpha VALUE           Nhân tau_ff riêng base, chỉ chỉnh khi chọn base
  test DEG [MOVE HOLD]  Đi thêm DEG độ, giữ, quay về HOME; mặc định 6 / 3 s
  save                  Lưu bộ số vào YAML trên máy
  next                  Chuyển 3 -> 2 -> 1 và in launch phần cứng tương ứng
  help / quit
QE/QI/QV/R phải hữu hạn và > 0; chỉ áp dụng khi test tiếp theo.
max_tau là thang chuẩn hóa R, không thay trần torque của URDF/tau_scale.
Hai khớp còn lại giữ position. Mỗi test tạo lại K theo quỹ đạo đang thử.
Ctrl-C trong test: trả về position rồi lưu log và mở đồ thị.
"""
BRYSON = ('max_e', 'max_int_e', 'max_tau')


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Giá trị phải hữu hạn và > 0')
    return value


def validate_joint_profile(params):
    if params.get('position_tracking_only') is not False:
        raise ValueError('Tune từng khớp cần position_tracking_only=false để dùng Q vận tốc')
    if float(params['control_hz']) != 100.0:
        raise ValueError('Tune phần cứng dùng control_hz=100')
    if params.get('use_discrete_lqr') is not True or params.get('require_discrete_stable') is not True:
        raise ValueError('Cần use_discrete_lqr=true và require_discrete_stable=true')
    if not 0 < positive(params['tau_scale']) <= 1:
        raise ValueError('tau_scale phải trong (0, 1]')
    positive(params.get('base_ff_alpha', 1.0))
    if params.get('feedback_model_mode', 'coupled') not in ('coupled', 'independent'):
        raise ValueError('feedback_model_mode phải là coupled hoặc independent')
    positive(params['gain_schedule_hz'])
    if params.get('gain_schedule_mode') != 'trajectory_time':
        raise ValueError('Tune từng khớp cần gain_schedule_mode=trajectory_time')
    scale = positive(params['tau_penalty_scale'])
    try:
        for e, ie, de, torque in zip(params['max_e'], params['max_int_e'],
                                      params['max_de'], params['max_tau']):
            for cost in (1 / positive(e)**2, 1 / positive(ie)**2,
                         1 / positive(de)**2, scale / positive(torque)**2):
                positive(cost)
    except (OverflowError, ZeroDivisionError) as error:
        raise ValueError('Trọng số Q/R ngoài phạm vi số hữu hạn') from error


class LqrJointTerminal:
    def __init__(self, params_file, joint='elbow', log_dir='results_new/lqr/joint_trials',
                 show_plot=True, dry_run=False, interface='can0', prestart_timeout=6.0):
        self.session = TuningSession('lqr', params_file)
        validate_joint_profile(self.parameters)
        self.parameters.setdefault('base_ff_alpha', 1.0)
        self.parameters.setdefault('feedback_model_mode', 'coupled')
        self.selected = JOINTS.index(joint)
        self.dry_run = dry_run
        self.interface = interface
        self.trial = RosLqrJointTrial(log_dir, show_plot, prestart_timeout)

    @property
    def parameters(self):
        return self.session.params

    def replace_parameter(self, key, value, vector=True):
        candidate = deepcopy(self.session.document)
        params = candidate['lqr_controller']['ros__parameters']
        if vector:
            params[key][self.selected] = positive(value)
        else:
            params[key] = positive(value)
        self.session.validate(candidate)
        validate_joint_profile(params)
        self.session.document = candidate

    def set_costs(self, qe, qi, r, qv=None):
        qe, qi, r = (positive(v) for v in (qe, qi, r))
        candidate = deepcopy(self.session.document)
        params = candidate['lqr_controller']['ros__parameters']
        values = (1 / math.sqrt(qe), 1 / math.sqrt(qi),
                  math.sqrt(params['tau_penalty_scale']) / math.sqrt(r))
        for key, value in zip(BRYSON, values):
            params[key][self.selected] = positive(value)
        if qv is not None:
            params['max_de'][self.selected] = positive(1 / math.sqrt(positive(qv)))
        self.session.validate(candidate)
        validate_joint_profile(params)
        self.session.document = candidate

    def costs(self):
        p, i = self.parameters, self.selected
        return (1 / p['max_e'][i]**2, 1 / p['max_int_e'][i]**2,
                p['tau_penalty_scale'] / p['max_tau'][i]**2)

    def show(self):
        print(f'feedback_model_mode={self.parameters["feedback_model_mode"]}')
        qe, qi, r = self.costs()
        qv = 1 / self.parameters['max_de'][self.selected]**2
        print(f'Khâu {self.selected + 1}: {JOINTS[self.selected]} | '
              f'QE={qe:g}, QI={qi:g}, QV={qv:g}, R={r:g}')
        print(f'base_ff_alpha={self.parameters["base_ff_alpha"]:g}, '
              f'tau_scale={self.parameters["tau_scale"]:g}, '
              f'slew={self.parameters["max_tau_rate_nm_s"][self.selected]:g} Nm/s')
        print(f'YAML lưu bằng save: {self.session.params_file}')

    def stage(self):
        self.show()
        print('Launch phần cứng (đặt/đỡ tay ở đúng zero trước startup):')
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
        elif command == 'set' and len(args) == 3:
            self.set_costs(*args)
            self.show()
        elif command == 'set' and len(args) == 4:
            qe, qi, qv, r = args
            self.set_costs(qe, qi, r, qv=qv)
            self.show()
        elif command in ('qe', 'qi', 'r') and len(args) == 1:
            costs = list(self.costs())
            costs[('qe', 'qi', 'r').index(command)] = positive(args[0])
            self.set_costs(*costs)
            self.show()
        elif command == 'qv' and len(args) == 1:
            self.set_costs(*self.costs(), qv=args[0])
            self.show()
        elif command in (*BRYSON, 'max_de') and len(args) == 1:
            self.replace_parameter(command, args[0])
            self.show()
        elif command == 'slew' and len(args) == 1:
            self.replace_parameter('max_tau_rate_nm_s', args[0])
            self.show()
        elif command == 'alpha' and len(args) == 1:
            if self.selected != 0:
                raise ValueError('alpha chỉ chỉnh khi đang chọn base')
            self.replace_parameter('base_ff_alpha', args[0], vector=False)
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
                line = input(f'lqr[khâu {self.selected + 1}/{JOINTS[self.selected]}]> ')
                if not self.execute(line):
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
                        'src/gim_arm_controller_lqr/config/lqr_joint_tuning.yaml')
    parser.add_argument('--log-dir', default='results_new/lqr/joint_trials')
    parser.add_argument('--interface', '--can', default='can0')
    parser.add_argument('--prestart-timeout', type=positive, default=6.0)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        LqrJointTerminal(args.params_file, args.joint, args.log_dir, not args.no_show,
                         args.dry_run, args.interface, args.prestart_timeout).run()
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
