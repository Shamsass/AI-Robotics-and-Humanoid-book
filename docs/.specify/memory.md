
# Agent Memory Management

The `OpenManusAgent` processes and stores its interaction history, or "memory", to facilitate learning and improvement. This is primarily handled by the `_convert_rollout_results_to_dataproto` method, which converts the results of an agent's interactions (a "rollout") into a structured data format.

## Rollout Results Conversion

The `_convert_rollout_results_to_dataproto` method takes the trajectory of agent and environment states, step rewards, and the final environment score, and packages them into a `RolloutResult` data protocol. This allows the agent's experiences to be efficiently stored and used for later analysis and training.

```python
def _convert_rollout_results_to_dataproto(
    trajectory: List[Tuple[Any, Any]],
    step_rewards: List[float],
    env_score: float,
) -> RolloutResult:
    """Converts the results of a rollout to a RolloutResult dataproto.

    Args:
        trajectory: A list of (agent_state, env_state) tuples.
        step_rewards: A list of rewards for each step.
        env_score: The final score of the environment.

    Returns:
        A RolloutResult dataproto.
    """
    rollout_result = RolloutResult()
    for agent_state, env_state in trajectory:
        rollout_result.trajectory.add().CopyFrom(
            trajectory_data.TrajectoryData(
                agent_state=agent_state,
                env_state=env_state,
            )
        )
    rollout_result.step_rewards.extend(step_rewards)
    rollout_result.env_score = env_score
    return rollout_result
```
