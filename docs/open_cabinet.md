# Projekt 2 - otwieranie szafki :file_cabinet:

### _(Zadanie przedstawiono do oceny na laboratoium trzecim w dniu 20.01.2022)_

 
## Treść zadania 📄   
W ramach drugiego projektu należało napisać program, który sprawi, że robot Velma otworzy drzwi szafki (jedne, o co najmniej 90 stopni), a następnie wróci do pozycji początkowej. 


## Etapy pracy nad projektem :construction_worker:

### 1) Utworzenie dedykowanego środowiska pracy w _Gazebo_  :earth_americas:
Dodano stolik (obiekt typu _static_) oraz szafkę stojącą na tymże stoliku (szafka nie jest typu _static_). Użyto szafki o nazwie **cabinet_door_fragile** z pakietu
_rcprg_gazebo_utils_, w folderze _data/gazebo/models_. Szafka ta ma wyłamywalne uchwyty, przez co konieczne jest wywieranie na nie ograniczonych sił. Do tworzenia własnego środowiska zastosowano _launch file_ `gazebo_world_editor.launch`, a następnie zapisano je w folderze _data/gazebo/worlds_ naszej przestrzeni roboczej. 

### 2) Skanowanie środowiska i utworzenie Octomapy :world_map:  
Aby zbudować mapę otoczenia uruchomiono odpowiedni węzeł poprzez plik _launch_ `roslaunch velma_common octomap_server.launch`. Mapę budowano poprzez ruszanie głową oraz korpusem. Serwer mapy zajętości buduje mapę zajętej przestrzeni oraz nieznanej przestrzeni - jest to niezbędne do planowania ruchu w trybie _JIMP_.  

### 3) Próby _"podchodzenia"_ do szafki z wykorzystaniem _CIMP_ :hammer_and_wrench:
Podniesienie ramienia manipulatora w okolice uchwytu szafki wykonano w trybie _CIMP_. Wynika to z faktu, że początkowa pozycja _Velmy_ (z rękami opuszczonymi wzdłuż korpusu) **jest dedykowaną konfiguracją początkową** dla zadania otwierania szafki. Zadanie docelowej pozycji końcówki prawego ramienia w okolicy uchwytu spowoduje ruch tego ramienia do góry _po łuku_. Po osiągnięciu docelowej pozycji ramie będzie gotowe do wykonania dalszych kroków (jego konfiguracja będzie krótko mówiąc _wygodna_). Próbowano zrealizować pierwszy etap (tj. zbliżenie się do uchwytu) z wykorzystaniem _JIMP_ oraz kinematyki odwrotnej, lecz prowadziło to do błędów w działaniu systemu. Wykonanie ruchu z wykorzystaniem _Plannera_ oraz odwrotnej kinematyki powodowało, że konfiguracja kątów w stawach prawego ramienia manipulatora nie była korzystna. Niektóre zmienne złączowe były blisko wartości granicznych co uniemożliwiało poprawne wykonanie dalszych ruchów.

### 4) Wykrywanie szafki i uchwytu :detective:
W tym etapie zadania **korzystamy z właściwości sterowania impedancyjnego i możliwości wchodzenia w kontakt z otoczeniem**, np. aby zlokalizować uchwyt drzwi, zadano trajektorię kolizyjną z małą prędkością, sztywnością i tolerancją uchybu (parametr _path_tol_).


### 5) Estymacja szerokości drzwi (_promienia szafki_)
Kiedy robot wsunął już palce chwytaka w uchwyt, wykonuje on niewielki ruch w przestrzeni kartezjańskiej. Pociąga za klamkę w swoją stronę na dystansie 5 cm. Ten ruch ma na celu rozpoznanie środowiska tzn. obiektu, który podlega manipulacji. Przed uchyleniem drzwi pobierana jest pozycja uchwytu w układzie bazy. Po uchyleniu pozycja uchwytu pobierana jest ponownie. Na podstawie informacji o położeniu i orientacji układu związanego z klamką przed i po uchyleniu estymowany jest promień szafki (szerokość drzwiczek). Do estymacji promienia wykorzystano _twierdzenie cosinusów_ - procedurę opisano w postaci schematu znajdującego się poniżej:
<p align="center"> 
  <img src="https://user-images.githubusercontent.com/80155305/150612062-620b0b57-d003-46c7-a8f9-de76f3dd6025.png"/>  
</p> 





Promień wyznaczany podczas uchylania drzwi jest zgodny z rzeczywistą szerokością szafki (zmierzoną w _Gazebo_).  
:warning: Przy opisanej metodzie wyznaczania promienia należy zaimplementować obsługę wyjątku **_ZeroDivisionError_**. W sterowaniu impedancyjnym pozycja osiągana jest z pewna dokładnościa. Może okazać się, że robot będzie myślał, że chwycił uchwyt, a w rzeczywistości tego nie zrobił. Przy uchylaniu szafki robot nie pociągnie za sobą drzwiczek - kąt _beta_ będzie równy 0 stopni i w mianowniku pojawi się zero.  

### 6) Generowanie trajektorii i ruch po okręgu
Po wyliczeniu szerokości drzwiczek generowana jest na tej podstawie trajektoria. Jest nią lista _n_ punktów (parametr _n_ może dobrać użytkownik) w układzie związanym z uchwytem szafki przed jej otwarciem. Z każdym punktem związana jest również orientacja chwytaka. Na przykład dla _n_ = 6 kolejne orientacje chwytaka to 0, 15, 30, 45, 60, 75, 90 stopni. Orientacja musi się zmianiać - w przeciwnym razie uchyt szafki zostałby wyrwany.

### 7) Bezpieczne puszczenie uchwytu i powrót do pozycji początkowej 
Po otworzeniu szafki do kąta 90 stopni wykonywane są dwa ruchy w przestrzeni kartezjańskiej. Chwytak najpierw wysuwa, a potem odsuwa się od drzwiczek. Na koniec ramię wraca do pozycji początkowej w trybie _JIMP_.



## Pliki źródłowe :card_file_box:
Finalna wersja programu sterującego robotem _Velma_ w zadaniu otwierania szafki znajduje sie w  _**manipulation/scripts/project_open.py**_ tego repozytorium.  
  
## Opis implementacji 💻 
Podczas prac nad zadaniem dołożono wszelkich starań, aby program spełniał następujące wymagania: 
* **odporność na zmiany stanu początkowego środowiska** ( nie ma _"zahardkowowanej"_ trajektorii - jest ona generowana na podstawie pozycji uchwytu oraz szerokości drzwiczek)
* **sprawdzanie powodzenia wykonania kolejnych etapów zadania i reakcja na sytuacje nieoczekiwanie** - w konsoli wyświetlają się podstawowe _logi_ pozwalające jednoznaczenie stwierdzić, na jakim etapie wykonania zadania aktualnie znajduje się robot.  
* **jakość kodu** - zastosowano podejście obiektowe. Trzy główne klasy to: _Initialization_, _CartSpace_ oraz _Hand_. Dodano także komentarze wyjaśniające, do czego służy dany fragment kodu. Nazwy funkcji dobierano tak, aby można było jednoznacznie i szybko stwierdzić, za co dana funkcja odpowiada.  


## Instrukcja uruchomienia 🚀   

1) W każdej konsoli należy wpisać: `source ~/mobile_ws/devel/setup.bash`
2) W osobnych konsolach wpisujemy kolejno poniższe komendy:  
```
roscore
roslaunch manipulation full_p2.launch
roslaunch manipulation rviz_and_objects_p2.launch
roslaunch rcprg_gazebo_utils gazebo_client.launch
rosrun manipulation project_open.py 
```
3) Przed kolejnym uruchomieniem należy wyczyścić pamięć współdzieloną: `rosrun velma_common reset_shm_comm.py`

## Testowanie systemu 🧪   
Przeprowadzone testy na zbudowanym środowisku wskazują, że system działa poprawnie. Poniżej załączono film poklatkowy przedstawiający robota _Velma_ w zadaniu otwierania szafki:
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150435294-bc33f67f-0494-445d-b143-80bb06a3f1ad.gif"/>  
</p> 


### Diagramy SysML :bar_chart:
Diagramy omawiające implementację, koncepcję oraz strukturę systemu umieszono w osobnym dokumencie :arrow_right: **[Link do diagramów](https://github.com/STERO-21Z/kaniuka-krasnodebski/blob/tiago/docs/projekt2_sysml.md)**













