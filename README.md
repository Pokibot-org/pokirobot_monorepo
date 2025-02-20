# pokirobot_monorepo
Repo contenant toutes les fichiers nécessaires pour réaliser le robot pokibot de la coupe de france de robotique.
 - brain_app: le soft central du robot, héberge la "stratégie".
 - legs_app: le soft s'occupant du control moteur.

# SETUP

Si vous voulez développer sur le soft du pokibot, initializez le repo comme ceci:

```sh
# create a new west workspace and pull the firmware
west init -m https://github.com/Pokibot-org/pokirobot_monorepo west-workspace
cd west-workspace/pokirobot_monorepo

# pull Zephyr upstream repository and modules (may take a while)
west update
```

cd west-workspace/pokirobot_monorepo

```sh
west-workspace/                     # contains .west/config
│
├── pokirobot_soft/                 # application firmware repository
│   ├── app/                        # application source files
│   ├── boards/                     # board specifications
│   ├── drivers/                    # additional drivers
│   ├── dts/                        # additional dts bindings
│   ├── tests/                      # unit test source files
│   └── west.yml                    # main manifest file
│
├── modules/                        # modules imported by Zephyr and CC firmware
|
├── tools/                          # tools used by Zephyr
│
└── zephyr/                         # upstream Zephyr repository
```

If you already have a west workspace set up, you can also re-use it to avoid having many copies of upstream Zephyr and modules:
```sh
# go to your workspace directory
cd your-zephyr-workspace

# pull the firmware
git clone https://github.com/Pokibot-org/pokirobot_monorepo

# re-configure and update the workspace
# (to be done each time you switch between applications in same workspace)
west config manifest.path pokirobot_monorepo
west update
```
# BUILD

Inside the brain_app or legs_app directory:
```sh
make build-debug
```

Or in release mode:
```sh
make build
```

## Sanity checks

```bash
./tools/check_eol.sh $(find src -name "*.c" -o -name "*.cpp" -o -name "*.h")
```
