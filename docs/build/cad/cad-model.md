# CAD Model

Botzo has been primarily designed using Fusion 360 CAD software. To support this,
we provide a `.f3z` file that can be imported directly into Fusion 360. Additionally,
`.step` files are available for users who prefer other CAD software.

Since Botzo is designed for 3D printing, all printable components are provided as
STL files. These are ready to be loaded into a slicer and printed using an FDM
printer.

All CAD files are available in the Botzo repository: [📁 View on GitHub](https://github.com/IERoboticsAILab/botzo/tree/main/CAD_files)

## About the Design

### The Leg

The leg design was inspired by the project [Chop](https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot/details)
by _Miguel Ayusto Parrilla_. By concentrating most of the weight in the shoulder,
this design reduces leg inertia, improving walking stability.

![Leg](../../assets/gifs/FULL_LEG.gif)

### The Body

The body is designed to house all internal components and cable routing while
leaving space on the dorsal side for mounting additional modules like a LiDAR or
webcams—supporting future capabilities like mapping and vision.

![Body](../../assets/gifs/animation_new_final_body.gif)
