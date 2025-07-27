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

# create_URDF
Documentation on how to build our URDF from STL file

A URDF is a file used i robotics to import the robot in a simulation. It uses the STL/CAD files as a base. Assemble them with rigid or rotatory joins, and save positions of each bone and pivot in the joints. To sum up, it is file that describe your robot in deep details and precision.

Download our URDF [here](https://github.com/botzo-team/create_URDF/blob/main/final_URDF.zip)

## Resources

Visit directly the repos if you don't understand something. This is a resume of what they do:
- [Webgraphviz](http://www.webgraphviz.com/)
- [Joint2Graphviz repo zip](https://github.com/yanshil/Joint2Graphviz)
- [Fusion2PyBullet repo zip](https://github.com/yanshil/Fusion2PyBullet)
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) <-- deprecated

## Prepare STL
As stated in the [Fusion2PyBullet repo](https://github.com/yanshil/Fusion2PyBullet), we must **prepare our STL** file properly.
1. All bodies/peaces/future links must be **components**.
2. Add physical materials
3. Add **joint** to connect all components. In fusion you can select various type of joint (static, rotational, etc...) and also the pivot point.
4. Call one componet as `base_link`
5. Remove links of comonents (if the are right clik on it and press Break Link)
    Result: https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/fusion_joints_one_leg.mp4
    ![result](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/fusion_joints_one_leg.gif)
6. Check STL joint-link structure using [Webgraphviz](http://www.webgraphviz.com/). Here how:
    - Download [Joint2Graphviz repo](https://github.com/yanshil/Joint2Graphviz) as a ZIP.
    - Extract the ZIP in a known directory.
    - Open terminal:

        ```powershell
        cd <path to Joint2Graphviz>

        Copy-Item ".\Joint2Graphviz-master\" -Destination "${env:APPDATA\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
        ```

        For me was:
        ```powershell
         cd C:\Users\orlan\OneDrive\Desktop\side_projects\Joint2Graphviz-master

         Copy-Item "..\Joint2Graphviz-master\" -Destination "..\..\..\..\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
        ```
    - Now the script to create a `.txt` file for your link graph is added to fusion
    - Open fusion
    - Open the file of the robot
    - Go to utility > Addins
    - Under `My Script` you should see `Joint2Graphviz`
    - Press it. It will create a `graph.txt` (see mine [here](https://github.com/botzo-team/create_URDF/blob/main/test_urdf_exporter/graph.txt))
    <p align="center">
        <img src="https://github.com/botzo-team/create_URDF/blob/main/test_urdf_exporter/graph_from_webgraphviz.png" alt="Graph Image" width="200"/>
    </p>

    - Copy and pase the text content to [Webgraphviz](http://www.webgraphviz.com/) and check your robot tf structure to be valid.

**(for more infors check [Joint2Graphviz readme](https://github.com/yanshil/Joint2Graphviz))**

-----

> [!WARNING]
> 7. We discovered that URDF does NOT suport close loop. Our leg design is one big close loop. You can check `wrong_graph` folder to see a graph with a close loop... The URDF won't work if there is a close loop

leg:

![urdf](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/urdf_leg.png)

Body:

![urfd](https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/urdf_body.png)

---

## Export URDF

1. Download [Fusion2PyBullet](https://github.com/yanshil/Fusion2PyBullet) as a ZIP
2. Extract the ZIP in a known directory.
3. Open terminal:

    ```powershell
    cd <path to Fusion2PyBullet>

    Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
    ```

    For me was:
    ```powershell
        PS C:\Users\orlan\OneDrive\Desktop\side_projects\Fusion2PyBullet-master> Copy-Item ".\Bullet_URDF_Exporter\" -Destination "..\..\..\..\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
    ```
4. Open fusion > utils > addins > my scripts > Bullet_URDF_Exporter
5. My URDF [here](https://github.com/botzo-team/create_URDF/blob/main/final_URDF.zip)

**(for more infors check [fusion2urdf readme](https://github.com/syuntoku14/fusion2urdf/blob/master/README.md) or [Fusion2PyBullet readme](https://github.com/yanshil/Fusion2PyBullet/blob/master/README.md))**