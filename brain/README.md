# Brain

Central runtime of the embodied system. Responsible for:

- Interpreting commands
- Task decomposition into Skills
- Planning under Cost constraints
- Reading/writing from Memory
- Calling LLM/VLM for reasoning

## 🧠 Includes:

- World model abstraction
- Model interfaces (e.g., via Ollama or DeepSeek)
- Prompt construction (JIT / dynamic)
- Skill orchestration logic

This is the decision-making and planning center of the system.


Test CMD as follow :

1. the environment is configured (see requirements.txt in the folder)

2. add api key in .env file

3. run the mcp client
```SHELL
python  path/to/brain-deepseek.py
```
