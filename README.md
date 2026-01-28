# qkrt-mcb

# Extra Notes

After cloning repo go to `qkrt-mcb-project`

Go to PipFile.Lock and remove the `pywin` section. Click SAVE.


Run

1. Install all required packages and the specific SCons version: This installs the missing libraries (pyelftools, jinja2, lxml) and downgrades SCons to version 3.x (since 4.0+ broke the build script)

`pipenv install pyelftools jinja2 lxml "scons<4.0"`

2. Refresh your shell's command cache: This forces your terminal to forget the system-wide SCons path (/usr/bin/scons) and find the one you just installed in your virtual environment.

`hash -r`

3. Run the build: Now that the paths are fixed and dependencies are met, you can use the standard command.

`scons build`

