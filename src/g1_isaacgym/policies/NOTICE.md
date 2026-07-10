`g1_legs_stand_walk_policy.pt` is `deploy/pre_train/g1/motion.pt` from
[unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
(commit `276801e`), copied verbatim. Unitree's repo is BSD-3-Clause licensed;
see that repo's `LICENSE` for the full text.

It's a TorchScript policy trained with legged_gym/IsaacGym for the stock
Unitree G1: 47-dim observation in, 12 leg joint-position residuals out. See
`../scripts/stand_g1.py` for the exact observation layout (angular velocity,
gravity vector, velocity command, leg joint pos/vel, last action, gait phase)
and how it's run here with a zero velocity command to stand in place.
