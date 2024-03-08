# Q&A

**Q: After following the instructions on the README to clone pyrfuniverse, I tried to run the test script in the Test directory and encountered an error.**

A: Did you forget to check the **README in the pyrfuniverse directory**? Please follow the guidance there to install other dependencies.

---

**Q: How to update rfuniverse?**

A: Download the new version of rfuniverse, **and run a scene with the script in the README at least once**. Clone the pyrfuniverse repository into your project folder. Then, in the terminal, run

```
pip install pyrfuniverse --upgrade
```

to update the pyrfuniverse library.

---

**Q: When running the test script, I encounter an error `ModuleNotFoundError: No module named 'extend'`.**

A: Try adding the following code snippet at the beginning of the test script:

```python
import sys
sys.path.append("/{path_to_rfuniverse}/pyrfuniverse")
```

(replace `{path_to_rfuniverse}` with the absolute path to the root directory of the project)

---

**Q: When running the test script, I encounter an error similar to the following:**

```
Unity Env Log Type:Exception
Condition:FileNotFoundException: File not found
StackTrace:System.Reflection.RuntimeMethodInfo.Invoke (System.Object obj, System.Reflection.BindingFlags invokeAttr, System.Reflection.Binder binder, System.Object[] parameters, System.Globalization.CultureInfo culture) (at <00000000000000000000000000000000>:0)
...
```

A: Try changing the terminal's working directory to `/{path_to_rfuniverse}/pyrfuniverse/Test`, rather than `/{path_to_rfuniverse}/pyrfuniverse`.

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