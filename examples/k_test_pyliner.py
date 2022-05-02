
import time
import pyliner


airliner = pyliner.Pyliner(**{"airliner_map": "cookiecutter.json",
                      "ci_port": 5009,
                      "to_port": 5012,
                      "script_name": "demo_test",
                      "log_dir": "./logs/"})


def callback_func(data):
    print data


airliner.subscribe({'name': '/Airliner/ES/HK', 'args':[{'name':'CmdCounter'}]}, callback_func)
