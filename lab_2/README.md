# kalinski_karpowicz

## Lab_2
Wizualizacja robota:
    roslaunch urdf_tutorial display.launch model:='$(find lab_2)/urdf/urdf_from_xacro.urdf'

Stworzenie modelu URDF z xacro:
     xacro xacro/test.xacro > urdf/urdf_from_xacro.urdf 


Odpalenie całości jako launchfile. Przed tym trzba zmienić ścieżkę do tabelki DH!!:
    roslaunch lab_2 lab2.launch
