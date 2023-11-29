# Motor Driver V2
- **Known Issue:** GNU Tools for STM32 (11.3.rel1) toolkit fails to compile in CubeIDE 1.13.1
  - Fix: Use GNU tools for STM32(10.3-2021.10) when using CubeIDE 1.13 or above
- **Do not** clone project into CubeIDE Workspace
  - Clone project into another folder and import **without** copying project into workspace
- Most serial terminals (`screen`, SerialTools, etc) do not support high enough baud on Mac
  - Use [`miniterm` in PySerial](https://pyserial.readthedocs.io/en/latest/tools.html#module-serial.tools.miniterm)

