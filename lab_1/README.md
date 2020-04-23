# kalinski_karpowicz

## Lab1
Pierwsze laboratorium znajduje się na branchu master.
Aby odpalić skrypt do sterowania żółwiem należy użyć polecenia roslaunch lab_1 lab1.launch

## wymagane paczki:
python2 pynput : https://pypi.org/project/pynput/
rospy
geometry_msgs

## Parametry:
~default_speed (double) 
~publish_rate (double)
~cmd_topic (string) Nazwa topicu na jaki mają być publikowane komendy sterowania
~front_key(string) Wybór klawisza jazdy do przodu
~back_key(string) Wybór klawisza jazdy do tyłu
~left_key(string) Wybór klawisza skrętu w lewo
~right_key(string) Wybór klawisza skrętu w prawo

## Tematy subskrybowane:
brak

## Tematy publikowane:
podany w parametrze ~cmd_topic

## Diagram
Aby utworzyć diagram należy uruchomić launch a następnie wywołać polecenie rosrun rqt_graph rqt_graph
