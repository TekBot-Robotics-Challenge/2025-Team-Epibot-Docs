# Documentation reading and lessons learned

We relied heavily on the Dofbot Jetson Nano official documentation provided by Yahboom: https://www.yahboom.net/study/Dofbot-Jetson_nano. The documentation was crucial from setup through to final validation.

At first the documentation felt dense and we got lost several times — we spent days searching for answers that were actually present in the docs. The key lesson: read carefully and follow the step-by-step sections in order.

Key sections that helped us
- "03/6. Virtual Machine Installation" — step-by-step instructions for creating the VM we used for the AI and MoveIt components.
- JupyterLab Course — we used JupyterLab to run interactive scripts on the robot and on the VM; it made iterative testing much faster than editing and running plain scripts.

Tips when using vendor documentation
- Follow the order the authors recommend (hardware first, then base OS, then ROS/MoveIt, then AI). Skipping steps will often lead to confusing errors.
- Keep a log of which tutorial steps you've completed and any deviations — we saved our command history and a short checklist per stage.
- If something is unclear, search for the exact error string (many common issues are covered by the community). For more complex problems, copy small snippets into a sandbox environment and test them independently.
