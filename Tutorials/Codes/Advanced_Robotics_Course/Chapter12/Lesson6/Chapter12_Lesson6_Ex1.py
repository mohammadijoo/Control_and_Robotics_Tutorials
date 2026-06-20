from stable_baselines3 import SAC

# Nominal dynamics (train)
train_env = PlanarReachEnv(render=False)

# Train SAC in simulation
model = SAC(
    policy="MlpPolicy",
    env=train_env,
    learning_rate=3e-4,
    buffer_size=200000,
    batch_size=256,
    gamma=0.99,
    tau=0.005,  # target smoothing
    train_freq=1,
    gradient_steps=1,
    policy_kwargs=dict(net_arch=[256, 256]),
    verbose=1,
)
model.learn(total_timesteps=200000)

# Transfer: perturbed dynamics (heavier links, more friction)
test_params = {
    "link_masses": [1.3, 1.3],
    "joint_damping": [0.08, 0.08],
    "joint_max_torque": 5.0,
}
test_env = PlanarReachEnv(render=False, dynamics_params=test_params)

def evaluate(env, model, n_episodes=20):
    returns = []
    for ep in range(n_episodes):
        obs, info = env.reset()
        done = False
        truncated = False
        ep_ret = 0.0
        while not (done or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, done, truncated, info = env.step(action)
            ep_ret += r
        returns.append(ep_ret)
    return np.mean(returns), np.std(returns)

J_sim_mean, J_sim_std = evaluate(train_env, model)
J_test_mean, J_test_std = evaluate(test_env, model)
print("J_sim:", J_sim_mean, "±", J_sim_std)
print("J_test:", J_test_mean, "±", J_test_std)
print("Delta J:", J_test_mean - J_sim_mean)
      
