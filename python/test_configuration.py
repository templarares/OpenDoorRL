#!/usr/bin/env python3

from mc_rtc_rl import Configuration, ConfigurationException

conf = Configuration("mc_rtc.yaml")
print(conf("MainRobot"))
try:
    print(conf("MainRobot")("NoSuchKey"))
except ConfigurationException as exc:
    pass

print("----")
test = conf.add("test")
test.add("bool", True)
test.add("int", 100)
test.add("float", 100.100)
test.add("bytes", b"Hello world!")
test.add("str", "你好")
test.add("tuple", (True, 42.42, "你好"))
test.add("list", [1.1, 2.2, 3.3])
test.add("dict", {"b": False, "i": -1, "float": -5.5, "str": "谢谢"})
array = test.array("array")
array.push(True)
array.push(-42.42)
array.push({"b": False, "i": -1, "float": -5.5, "str": "谢谢"})
test.add("test", test)
print(conf)
print("----")
