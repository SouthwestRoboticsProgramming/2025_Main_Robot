# Robot

## Building Pathfinding

Pathfinding requires Rust to build, and `cross` to deploy. You can install both
by following the directions below.

### On Windows:

1. Install WSL by opening Command Prompt and running `wsl --install`
2. Download and install Docker Desktop from [here](https://www.docker.com/products/docker-desktop/).
   Make sure the option to use WSL instead of Hyper-V is enabled.
3. Go to "Turn Windows features on or off" and ensure that `Virtual Machine Platform` and `Windows Subsystem for Linux`
4. Reboot your computer. (optional?)
5. Docker Desktop should open on next boot. If not, open it. Then accept the licence agreement.
6. Download the `rustup` installer from [here](https://rustup.rs) and run it. This should open a
   terminal window.
7. Follow the instructions to install Rust. If it prompts you to install Visual Studio, install it also.
8. Open Command Prompt and run `cargo install cross`
9. Using Command Prompt, navigate to the `Robot` folder and run `cargo build --release`
10. Hopefully everything should be working now but Windows is silly so who knows :)

### On Linux:

1. Install Rust via `rustup`: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
2. Install Docker Engine using the instructions [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
3. You may need to add yourself to the `docker` group to ensure your user has permission to use Docker:
   `sudo usermod -aG docker $USER`, then either reboot or run `newgrp docker`.
4. Install `cross`: `cargo install cross`