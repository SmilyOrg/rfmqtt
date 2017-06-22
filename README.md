rfmqtt - RF 433MHz to MQTT bridge.

# Compiling

1. Change your working directory to the project directory.
2. Create a build directory, e.g. `mkdir build`.
3. Change to the build directory, e.g. `cd build`.
4. Use CMake to create project files, e.g. `cmake ..` to create a `Makefile` or
`cmake -GNinja ..` to create a Ninja build file.
5. Compile the code, e.g. `make -j4` or `ninja`. This takes several minutes on
a Raspberry Pi 2.
6. After successful compilation, the `rfmqtt` executable should be ready and
waiting in the build directory.

For everyday use, you might want to build as a `Release` or `RelWithDebInfo`
build type to be more efficient on resources.

# Setup

1. Download or compile the `rfmqtt` binary and example `config.yaml`.
2. Configure your MQTT connection, RF devices and mappings in `config.yaml`.

# Installing (optional)

## As a CLI app

1. Copy `rfmqtt` to `/usr/local/bin` to install `rfmqtt` to be used globally,
e.g. `cp rfmqtt /usr/local/bin`.
2. Copy `config.yaml` to `~/.config/rfmqtt/config.yaml` for it to be picked up
automatically, e.g. `cp config.yaml ~/.config/rfmqtt/`.

The config file will also get picked up in the working directory and other
dirs defined by the "XDG Base Directory Specification". You can also use the
`--config` command line option to set a custom path.

## As a service
1. Copy `rfmqtt` to `/usr/local/bin` if you want to use the default
service configuration, e.g. `cp rfmqtt /usr/local/bin`.
2. Copy `config.yaml` to `/etc/xdg/rfmqtt/config.yaml` for it to be picked up
automatically,
e.g. `mkdir /etc/xdg/rfmqtt; cp config.yaml /etc/xdg/rfmqtt/`.
3. Copy `service/rfmqtt.service` to `/etc/systemd/system/`,
e.g. `sudo cp service/rfmqtt.service /etc/systemd/system/`.
4. Run `sudo systemctl daemon-reload` to refresh services. This should be done
every time you change the `.service` file.

# Running

## As a program

1. Run `rfmqtt` if installed or `./rfmqtt` to run from working directory.

See `./rfmqtt -h` for more info on command line arguments.

## As a service

1. Install as a service if you haven't already.
2. Run `sudo systemctl start rfmqtt.service` to start the service.
3. Now it should always run in the background automatically.

You can use `status` in place of `start` to see the status of the service.
Also useful are `stop` and `restart`.