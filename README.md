# RVO

This is a modified implementation of [MengGuo's](https://github.com/MengGuo/RVO_Py_MAS) Reciprocal Velovity Obstacle (RVO) for Multi-agent systems. This version has been made to simulate differential-wheeled robots and keep track of total flow time. It was prepared as part of the Mobile Robot Systems Miniproject titled Adversarial Velocity Obstacles. As such, it investigates how agents could act selfishly by dishonestly broadcasting false velocities. It also includes an implementation of a mitigation scheme: having the agents infer the other agent's velocities based on observations rather than trust them.

To kick start the visualization for different starting scenarios under different (dishonest and honest broadcast as well as inference) velocity sharing schemes, run:

```
./run.sh
```

To just print out the malicious agent's and total flow times, for different starting scenarios, without visualization, run:

```
python3 example.py --gen_breakdown=True
```

---

## References

- The paper on on [RVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf)
- MengGuo's [original implementation](https://github.com/MengGuo/RVO_Py_MAS)
