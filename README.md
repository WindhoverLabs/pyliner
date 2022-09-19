# Pyliner

Pyliner is a Python 2.7.x package for sending Airliner commands to a drone
using a user-friendly scripting interface.

# To Run

```
make virtual-env
make run
```

# IDE Notes
- Set the working directory to `airliner_old/tools/pyliner` in PyCharm



1. cd software/airliner/public

2. git checkout pyliner_commands

3. git submodule update --init --recursive

4. make quad-sitl

5. make quad-sitl-workspace

6.  cd build/multirotor/quad/

7. terminator -g term-sitl.cfg

8. Add "PX4_Vechicle_Global_Position" to downlink

9. Open a new shell

10. Kill yamcs. At the moment pyliner will use the same port number as YAMCS. And we also don't have TO channels.

11. cd core/tools/pyliner

12. make virtual-env

13. source venv/bin/activate

