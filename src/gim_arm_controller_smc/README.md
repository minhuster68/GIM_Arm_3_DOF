# SMC từng khớp trên tay thật

Hướng dẫn đầy đủ cho người tune nằm tại
[README_TUNE_SMC.md](../../README_TUNE_SMC.md), ở đầu workspace.

Workspace `/home/mirabo/GIM_Arm_3_DOF`, branch `smc-test`.
Terminal: `bash tools/tune_smc_terminal.sh --joint elbow`.
Config: `config/smc_joint_tuning.yaml`; thứ tự vector base, shoulder, elbow.
Đây là bộ khởi đầu chưa tune. Tune elbow → shoulder → base, hai khớp còn lại
giữ position. Đọc hướng dẫn zero, HOME và đổi khớp trước khi chạy phần cứng.
