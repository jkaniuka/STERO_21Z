# Projekt 1 - zadanie _Pick & Place_ 

  
### _(Zadanie przedstawiono do oceny na laboratoium drugim w dniu 16.12.2021)_

 
## Treść zadania 📄   
Celem projektu jest napisanie programu sterującego robotem Velma, który spowoduje, że robot chwyci, przeniesie i odłoży obiekt w inne miejsce.    
Należy napisać program, dzięki któremu Velma:
* chwyci obiekt _object1_ stojący na pierwszym stoliku _table1_
* przeniesie go na drugi stolik _table2_
* odłoży go na drugim stoliku, tj. zwolni chwyt i wycofa ramię w pozycję początkową  
_(Należy zastosować planowanie ruchu, aby robot omijał przeszkody podczas ruchu)_


## Etapy pracy nad projektem 🤔 
Pracę nad projektem rozpoczęto już podczas pierwszych zajęć laboratoryjnych z bloku manipulacyjnego. Podczas laboratorium udało się zrealizować dwa, początkowe etapy proejktu:   

### 1) Utworzenie dedykowanego środowiska pracy w _Gazebo_ 🪑 
Dodano dwa stoliki o nazwach _table1_ i _table2_ - przez stolik rozumiemy coś, na czym można położyć obiekt, stoliki są obiektami typu _static_, czyli nieruchomymi. Stoliki są w takiej odległości od robota, aby nadmiernie nie ograniczać jego ruchu.  
Do tworzenia własnego środowiska zastosowano _launch file_ `gazebo_world_editor.launch`, a następnie zapisano je w folderze _data/gazebo/worlds_ naszej przestrzeni roboczej. Kolejne etapy tworzenia _**octomapy**_ przedstawiono poniżej:  
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/146288933-5641c309-f42c-4290-93c3-f587daa678fe.png" width="300" height="300"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146289113-176a2c1a-372f-4b57-ac8b-7932fd729137.png" width="300" height="300"/> 
  <img src="https://user-images.githubusercontent.com/80155305/146289123-7b7dcdf2-ae73-4c4c-92f9-91824ab36766.png" width="300" height="300"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146289139-1b3cb1a3-802b-4b52-adeb-3a42b6e10180.png" width="300" height="300"/> 
  <img src="https://user-images.githubusercontent.com/80155305/146289145-21b5b876-11c4-483d-b815-280b4cdcbf53.png" width="300" height="300"/> 
</p> 


### 2) Zapoznanie się ze skryptami testowymi z pakietu _velma_task_cs_ros_interface_ 
Podczas implementacji węzła odpowiedzialnego za wykonanie zadanie _Pick&Place_ intensywnie wykorzystywano przykładowe pliki zawierające przykłady konkretnych zachowań robota. Pomocne pliki to między innymi:  _test_head.py_, _test_cimp_pose.py_, _test_grippers.py_.


### 3) Skanowanie środowiska i utworzenie Octomapy 🗺️   
Aby zbudować mapę otoczenia uruchomiono odpowiedni węzeł poprzez plik _launch_ `roslaunch velma_common octomap_server.launch`. Mapę budowano poprzez ruszanie głową oraz korpusem. Serwer mapy zajętości buduje mapę zajętej przestrzeni oraz nieznanej przestrzeni - jest to niezbędne do planowania ruchu w trybie _JIMP_.  

### 4) Ruch do punktu zadanego w przestrzeni konfiguracyjnej (**JIMP**)
Zbliżenie się końcówki manipulatora do słoika to długi i skomplikowany ruch, więc zastosowano _Planner_ - wykonywanie skomplikowanych ruchów (dużych zmian w konfiguracji) **wymaga** stosowania planowania trajektorii. _Planner_ bierze pod uwagę strukturę kinematyczną robota wraz ze wszystkimi ograniczeniami, oraz model środowiska. Dzięki temu może wygenerować bezkolizyjną trajektorię lub podać informację o błędzie i jego przyczynie.

### 5) Próby _"podchodzenia"_ do obiektu z wykorzystaniem CIMP 🛠️ 
Należy korzystać z ruchów w przestrzeni końcówki (_cart_imp_) do wykonywania mniejszych ruchów, np. zbliżanie ręki do chwytanego obiektu. W tym przypadku nie ma możliwości stosowania planowania. Robot zbliża się do słoika na pewną odległość z zamkniętym chwytakiem, następnie otwiera chwytak i podchodzi do obiektu zadając ruch w przestrzeni końcówki.

### 6) Wprowadzenie odwrotnej kinematyki (_InvKin_)
Wykorzystaliśmy w tym celu klasę liczącą odwrotną kinematykę (_**KinematicsSolverVelma**_).  Poza budową mapy otoczenia nie korzystamy z żadnych bardziej złożonych algorytmów rozpoznawania obrazów, więc informację o położeniu obiektów ( potrzebną do wyliczenia odwrotnej kinematyki) pobieramy bezpośrednio z symulatora _Gazebo_. Wykorzystano w tym celu odpowiedni plik launch - `gazebo_publish_ros_tf_object.launch`. Rozwiązanie zadania odwrotnej kinematyki nie jest problemem trywialnym szczególnia dla manipulatora redundantnego o 7DOF jakim są ramiona LWR4+ robota _Velma_. Podanie do _Solvera_ tylko jednego punktu docelowego jest wysoce ryzykowne :warning:. Może okazać się, że rozwiązanie nie zostanie znaleźione i robot będzie musiał przerwać wykonywanie zadania.  
  
Przyjęliśmy następujący sposób postępowania :arrow_down: :
* punktem pobieranym z symulatora _Gazebo_ jest środek słoika znajdujący się w centralnym punkcie podejnowanego obiektu
* wokół wspomnienego punktu generujemy okrąg o promieniu, który jest parametrem dostrajalnym i był modyfikowany na etapach testów
* na okręgu generujemy wiele układów współrzędnych, aby zapewnić mnogość możliwośći podejścia do obiektu
* przed przekazaniem punktów docelowych do _Plannera_ dokonujemy **filtracji** tzn. odrzucamy te punkty, dla których docelowe wartości zmiennych złączowych w stawach manipulatora są bliskie wartościom granicznym (ułatwi to pracę _Plannerowi_). 

### 7) Integracja systemu w całość 👌 
Na koniec ustalono kolejność wywoływania zaimplementowanych funkcji oraz dodano obsługę błędów.

## Pliki źródłowe :card_file_box:
Finalna wersja programu sterującego robotem _Velma_ w zadaniu Pick&Place znajduje sie w katalogu _**manipulation/pick_and_place**_ tego repozytorium.  
  
## Opis implementacji 💻 
Podczas prac nad zadaniem dołożono wszelkich starań, aby program spełniał następujące wymagania: 
* **odporność na zmiany stanu początkowego środowiska** ( nie ma _"zahardkowowanych"_ pozycji obiektów, wykorzystujemy odwrotną kinematykę)
* **korzystanie z _octomapy_ i planowanie** - na początku zbudowano _octomapę_ i korzystano z niej w trybie _offline_, aby nie obciążać zbytnio procesora komputera. Program dopuszcza budowanie nowej mapy poprzez wywołanie skryptu `test_custom_head.py`. 
* **sprawdzanie powodzenia wykonania kolejnych etapów zadania i reakcja na sytuacje nieoczekiwanie** - w konsoli wyświetlają się podstawowe _logi_ pozwalające jednoznaczenie stwierdzić, na jakim etapie wykonania zadania aktualnie znajduje się robot.  
* **jakość kodu** - kod podzielono na zbiór pomniejszych funkcji, odpowiedzialnych za wykonanie danej akcji/fragmentu zachowania. Dodano także komentarze wyjaśniające, do czego służy dany fragment kodu. Nazwy funkcji dobierano tak, aby można było jednoznacznie i szybko stwierdzić, za co dana funkcja odpowiada.  

## Diagram przejść 📊 

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/146396792-62452a54-0466-4a02-aabb-0152626a1035.png"/>  
</p> 


## Instrukcja uruchomienia 🚀   

1) W każdej konsoli należy wpisać: `source ~/mobile_ws/devel/setup.bash`
2) W osobnych konsolach wpisujemy kolejno poniższe komendy:  
`roscore`    
`roslaunch manipulation full.launch`    
`roslaunch manipulation rviz_and_objects.launch`    
`roslaunch rcprg_gazebo_utils gazebo_client.launch`    
`rosrun manipulation pick_and_place.py`  


3) Przed kolejnym uruchomieniem należy wyczyścić pamięć współdzieloną: `rosrun velma_common reset_shm_comm.py`

## Testowanie systemu 🧪   
Przeprowadzone testy na zbudowanym środowisku wskazują, że system działa poprawnie. Poniżej przedstawiono kolejne etapy wykonywania zadania _Pick&Place_ przez robota Velma: 

1) Budowa mapy (przedstawiono już wcześniej)  
2) Ruch w kierunku obiektu
3) Zbliżanie się do obiektu
4) Chwycenie obiektu
5) Transport obiektu na drugi stół
6) Odstawienie obiektu  
7) Powrót do pozycji początkowej 

:warning: Podczas pracy czasami występowały dwa (_niezależne od nas_) błędy, które utruniały działanie systemu sterowania i wpływały na poprawność wykonania zadania:  
1) Chwytaki nie zawsze sie zamykały (ma to miejsce również w pracy z rzeczywistym robotem).
2) Manipulator zatrzymywał się i informował o kolizji z otoczeniem, w której _de facto_ nie był. Spowodowane jest to nieuwzględnianiem przez _Planner_ jednego z elementów końcówki robota widocznego na grafice poniżej:  
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/146643562-398a3ceb-3a42-41c4-85b1-29a62e9a2d39.png" width="300" height="300"/>  
</p>   



Wizualizacja kolejnych etapów wykonywania zadania z poziomu symulatora _Gazebo_:  
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/146333187-ef38c63f-e08f-4b44-be36-64c191d08e0c.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333229-bd526dab-c26c-48eb-a0d9-ac812f97ce00.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333250-f24e7f7a-d595-4e39-a2f3-4a36fc0bc485.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333282-9929ab01-c5e9-4220-b62e-a9a9ee004b3d.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333300-6319e65d-63f6-4ab1-b6f2-377e3f08215d.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333329-514ecb4a-b8fb-450b-ae17-8010c11f2d28.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333354-6cc4c40f-f780-4e53-980c-7036c65a3122.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333424-e40285ce-52fe-4746-8a2d-12396425597a.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333446-6559093d-f828-4c21-b5d7-fa1e3071d0c3.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333464-203569bf-7f1c-4087-8c69-45f18fb9690d.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333503-2561cd86-13f4-4428-8033-c27f1cb7b322.png" width="450" height="400"/>  
  <img src="https://user-images.githubusercontent.com/80155305/146333521-3c1c1069-0c38-4c23-a239-1b9a90dc76c1.png" width="450" height="400"/>  
</p> 








