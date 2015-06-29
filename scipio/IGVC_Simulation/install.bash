rosdep update

rosdep install --from-paths ./ --ignore-src --rosdistro ${ROS_DISTRO} -y

echo "Now grabbing scipio's gazebo meshes from the EDT wiki..."
cd scipio_simulation/meshes

if [ -e ./Back_left_wheel_link.STL.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/7/70/Back_left_wheel_link.STL.zip
    unzip Back_left_wheel_link.STL.zip
    rm Back_left_wheel_link.STL.zip
else
    if [ ! -e ./Back_left_wheel_link.STL ]; then
        wget http://wiki.chicagoedt.org/images/7/70/Back_left_wheel_link.STL.zip
        unzip Back_left_wheel_link.STL.zip
        rm Back_left_wheel_link.STL.zip
    fi
fi

if [ -e ./Base_link.DAE.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/7/71/Base_link.DAE.zip
    unzip Base_link.DAE.zip
    rm Base_link.DAE.zip
else
    if [ ! -e ./Base_link.DAE ]; then
        wget http://wiki.chicagoedt.org/images/7/71/Base_link.DAE.zip
        unzip Base_link.DAE.zip
        rm Base_link.DAE.zip
    fi
fi

if [ -e ./Front_left_wheel_link.STL.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/9/9b/Front_left_wheel_link.STL.zip
    unzip Front_left_wheel_link.STL.zip
    rm Front_left_wheel_link.STL.zip
else
    if [ ! -e ./Front_left_wheel_link.STL.zip ]; then
        wget http://wiki.chicagoedt.org/images/9/9b/Front_left_wheel_link.STL.zip
        unzip Front_left_wheel_link.STL.zip
        rm Front_left_wheel_link.STL.zip
    fi
fi


if [ -e ./Back_right_wheel_link.DAE.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/9/95/Back_right_wheel_link.DAE.zip
    unzip Back_right_wheel_link.DAE.zip
    rm Back_right_wheel_link.DAE.zip
else
    if [ ! -e ./Back_right_wheel_link.DAE.zip ]; then
        wget -c http://wiki.chicagoedt.org/images/9/95/Back_right_wheel_link.DAE.zip
        unzip Back_right_wheel_link.DAE.zip
        rm Back_right_wheel_link.DAE.zip
    fi
fi

if [ -e ./Back_right_wheel_link.STL.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/a/aa/Back_right_wheel_link.STL.zip
    unzip Back_right_wheel_link.STL.zip
    rm Back_right_wheel_link.STL.zip
else
    if [ ! -e ./Back_right_wheel_link.STL.zip ]; then
        wget http://wiki.chicagoedt.org/images/a/aa/Back_right_wheel_link.STL.zip
        unzip Back_right_wheel_link.STL.zip
        rm Back_right_wheel_link.STL.zip
    fi
fi

if [ -e ./Front_right_wheel_link.STL.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/d/d0/Front_right_wheel_link.STL.zip
    unzip Front_right_wheel_link.STL.zip
    rm Front_right_wheel_link.STL.zip
else
    if [ ! -e ./Front_right_wheel_link.STL.zip ]; then
        wget http://wiki.chicagoedt.org/images/d/d0/Front_right_wheel_link.STL.zip
        unzip Front_right_wheel_link.STL.zip
        rm Front_right_wheel_link.STL.zip
    fi
fi

if [ -e ./Front_left_wheel_link.DAE.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/d/d4/Front_left_wheel_link.DAE.zip
    unzip Front_left_wheel_link.DAE.zip
    rm Front_left_wheel_link.DAE.zip
else
    if [ ! -e ./Front_left_wheel_link.DAE.zip ]; then
        wget http://wiki.chicagoedt.org/images/d/d4/Front_left_wheel_link.DAE.zip
        unzip Front_left_wheel_link.DAE.zip
        rm Front_left_wheel_link.DAE.zip
    fi
fi

if [ -e ./Base_link.STL.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/1/10/Base_link.STL.zip
    unzip Base_link.STL.zip
    rm Base_link.STL.zip
else
    if [ ! -e ./Base_link.STL.zip ]; then
        wget http://wiki.chicagoedt.org/images/1/10/Base_link.STL.zip
        unzip Base_link.STL.zip
        rm Base_link.STL.zip
    fi
fi

if [ -e ./Front_right_wheel_link.DAE.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/f/fa/Front_right_wheel_link.DAE.zip
    unzip Front_right_wheel_link.DAE.zip
    rm Front_right_wheel_link.DAE.zip
else
    if [ ! -e ./Front_right_wheel_link.DAE.zip ]; then
        wget http://wiki.chicagoedt.org/images/f/fa/Front_right_wheel_link.DAE.zip
        unzip Front_right_wheel_link.DAE.zip
        rm Front_right_wheel_link.DAE.zip
    fi
fi

if [ -e ./Back_left_wheel_link.DAE.zip ]; then
    wget -c http://wiki.chicagoedt.org/images/5/5a/Back_left_wheel_link.DAE.zip
    unzip Back_left_wheel_link.DAE.zip
    rm Back_left_wheel_link.DAE.zip
else
    if [ ! -e ./Back_left_wheel_link.DAE.zip ]; then
        wget http://wiki.chicagoedt.org/images/5/5a/Back_left_wheel_link.DAE.zip
        unzip Back_left_wheel_link.DAE.zip
        rm Back_left_wheel_link.DAE.zip
    fi
fi

cd ../../

cd grass/materials/textures

if [ -e ./Igvc_lines_4000x4000cm.png ]; then
    wget -c http://wiki.chicagoedt.org/images/6/63/Igvc_lines_4000x4000cm.png
    mv Igvc_lines_4000x4000cm.png igvc_lines_4000x4000cm.png
else
    if [ ! -e ./Igvc_lines_4000x4000cm.png ]; then
        wget http://wiki.chicagoedt.org/images/6/63/Igvc_lines_4000x4000cm.png
        mv Igvc_lines_4000x4000cm.png igvc_lines_4000x4000cm.png
    fi
fi

cd ../../meshes/images

if [ -e ./0_Grass0081_L.jpg ]; then
    wget -c http://wiki.chicagoedt.org/images/c/c8/0_Grass0081_L.jpg
else
    if [ ! -e ./0_Grass0081_L.jpg ]; then
        wget http://wiki.chicagoedt.org/images/c/c8/0_Grass0081_L.jpg
    fi
fi

if [ -e ./1_igvc_lines.png ]; then
    wget -c http://wiki.chicagoedt.org/images/c/c9/1_igvc_lines.png
else
    if [ ! -e ./1_igvc_lines.png ]; then
        wget http://wiki.chicagoedt.org/images/c/c9/1_igvc_lines.png
    fi
fi

cd ../../../ 


echo "Finished Installing IGVC Simulation Repository."
