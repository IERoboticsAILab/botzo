<div align="center">
<h1>BOTZO 🐾</h1>

**`The good boy quadruped robot :)`**

<p align="center">
    <a href="https://www.instagram.com/botzo.ie/" target="_blank" rel="noopener noreferrer">
        <img alt="Instagram" src="https://img.shields.io/badge/Instagram-%232C3454.svg?style=for-the-badge&logo=Instagram&logoColor=white" />
    </a>
    <a href="" target="_blank" rel="noopener noreferrer">
        <img alt="LinkedIn" src="https://img.shields.io/badge/Youtube-%232C3454.svg?style=for-the-badge&logo=Youtube&logoColor=white" />
    </a>
    <a href="mailto:botzoteam@gmail.com">
        <img alt="Gmail" src="https://img.shields.io/badge/Gmail-2c3454?style=for-the-badge&logo=gmail&logoColor=white" />
    </a>
    <a href="">
        <img alt="Views" src="https://komarev.com/ghpvc/?username=botzo&color=blue&style=for-the-badge&abbreviated=true" />
    </a>

</p>

</div>

more [here](https://github.com/IERoboticsAILab/botzo)

# Reinforcement Learning
In this repo we will explore how we teach botzo to walk

## Resources

- Genesis [link](https://genesis-world.readthedocs.io/en/latest/user_guide/index.html)
- Basics [link](https://youtu.be/f6LkEQsXGF8?si=eqC7IzNiBTZROUgm)
- theconstrucsim course Intermediate Generative AI for Robotics [link](https://app.theconstruct.ai/courses/intermediate-generative-ai-for-robotics-276/)
- theconstrucsim course Mastering Reinforcement Learning for Robotics [link](https://app.theconstruct.ai/courses/mastering-reinforcement-learning-for-robotics-286/)
- [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/index.html)

# Installation Isaac Sim

Computer specifications:
- RAM: 64GB
- GPU: RTX 4090
- VRAM: 16GB
- OS: Windows 11
- Storage: 1TB
>_Check system requironments compatibility and satisfaction [here](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-comp-check%404.5.0-rc.6%2Brelease.675.f1cca148.gl.windows-x86_64.release.zip)_

Downlaod [IsaacSim](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.windows-x86_64.release.zip) for your OS

```shell
mkdir C:\isaacsim
cd %USERPROFILE%/Downloads
tar -xvzf "isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.windows-x86_64.release.zip" -C C:\isaacsim
cd C:\isaacsim
post_install.bat
isaac-sim.selector.bat
```
> [!NOTE] 
>The Isaac Sim app can be run directly via command line with isaac-sim.bat. Start new empty simulation.

> [!NOTE] 
>_extension_examples_ and _standalone_examples_ folder for the tutorials

## Getting started with Isaac Sim

Documentation [here](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/introduction/quickstart_index.html#isaac-sim-intro-quickstart-series)