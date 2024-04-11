# Q&A

---

**Q: When running `test_heat_map.py` file on Linux and generating a heat map, I encounter the following error:**

```bash
Unity Env Log Type:Exception
Condition:DllNotFoundException: Unable to load DLL 'gdiplus'. Tried to load the following dynamic libraries: Unable to load dynamic library 'gdiplus' because of 'Failed to open the requested dynamic library (0x06000000) dlerror() = gdiplus: cannot open shared object file: No such file or directory
```

A: Try running the following code in the terminal to install the missing library files:

```bash
sudo apt install libc6-dev
sudo apt install libgdiplus
```

---

**Q: After creating an env with the pyrfuniverse script, the RFU simulation program is not invoked. How can I solve this?**

A: The last run simulation program is invoked by default. If you have never run a simulation program or the last run was in the Unity Editor, it will not be automatically invoked. Double-click to start Release once.

---

**Q: Why does using the SetTransform interface to set an object's position not change the data["position"]?**

A: After calling the interface, you need to call env.step() to execute the simulation and make the interface effective.

---

**Q: How to choose the correct version?**

A: RFU-Release and pyrfu are released with synchronized version numbers. Compatibility is guaranteed when the first three digits of the version number match. The last digit indicates a patch fix version; it is recommended to upgrade when there is an update.

---

**Q: How to deal with gym installation errors when installing an older version?**

A: You can install it by rolling back the versions of setuptools and wheel

```
pip install setuptools==65.5.0

pip install --user wheel==0.38.0
```

---