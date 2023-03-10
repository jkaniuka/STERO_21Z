# Projekt 2 - otwieranie szafki :file_cabinet:

### _(Zadanie przedstawiono do oceny na laboratoium trzecim w dniu 20.01.2022)_

 
## Tre艣膰 zadania 馃搫   
W ramach drugiego projektu nale偶a艂o napisa膰 program, kt贸ry sprawi, 偶e robot Velma otworzy drzwi szafki (jedne, o co najmniej 90 stopni), a nast臋pnie wr贸ci do pozycji pocz膮tkowej. 


## Etapy pracy nad projektem :construction_worker:

### 1) Utworzenie dedykowanego 艣rodowiska pracy w _Gazebo_  :earth_americas:
Dodano stolik (obiekt typu _static_) oraz szafk臋 stoj膮c膮 na tym偶e stoliku (szafka nie jest typu _static_). U偶yto szafki o nazwie **cabinet_door_fragile** z pakietu
_rcprg_gazebo_utils_, w folderze _data/gazebo/models_. Szafka ta ma wy艂amywalne uchwyty, przez co konieczne jest wywieranie na nie ograniczonych si艂. Do tworzenia w艂asnego 艣rodowiska zastosowano _launch file_ `gazebo_world_editor.launch`, a nast臋pnie zapisano je w folderze _data/gazebo/worlds_ naszej przestrzeni roboczej. 

### 2) Skanowanie 艣rodowiska i utworzenie Octomapy :world_map:  
Aby zbudowa膰 map臋 otoczenia uruchomiono odpowiedni w臋ze艂 poprzez plik _launch_ `roslaunch velma_common octomap_server.launch`. Map臋 budowano poprzez ruszanie g艂ow膮 oraz korpusem. Serwer mapy zaj臋to艣ci buduje map臋 zaj臋tej przestrzeni oraz nieznanej przestrzeni - jest to niezb臋dne do planowania ruchu w trybie _JIMP_.  

### 3) Pr贸by _"podchodzenia"_ do szafki z wykorzystaniem _CIMP_ :hammer_and_wrench:
Podniesienie ramienia manipulatora w okolice uchwytu szafki wykonano w trybie _CIMP_. Wynika to z faktu, 偶e pocz膮tkowa pozycja _Velmy_ (z r臋kami opuszczonymi wzd艂u偶 korpusu) **jest dedykowan膮 konfiguracj膮 pocz膮tkow膮** dla zadania otwierania szafki. Zadanie docelowej pozycji ko艅c贸wki prawego ramienia w okolicy uchwytu spowoduje ruch tego ramienia do g贸ry _po 艂uku_. Po osi膮gni臋ciu docelowej pozycji ramie b臋dzie gotowe do wykonania dalszych krok贸w (jego konfiguracja b臋dzie kr贸tko m贸wi膮c _wygodna_). Pr贸bowano zrealizowa膰 pierwszy etap (tj. zbli偶enie si臋 do uchwytu) z wykorzystaniem _JIMP_ oraz kinematyki odwrotnej, lecz prowadzi艂o to do b艂臋d贸w w dzia艂aniu systemu. Wykonanie ruchu z wykorzystaniem _Plannera_ oraz odwrotnej kinematyki powodowa艂o, 偶e konfiguracja k膮t贸w w stawach prawego ramienia manipulatora nie by艂a korzystna. Niekt贸re zmienne z艂膮czowe by艂y blisko warto艣ci granicznych co uniemo偶liwia艂o poprawne wykonanie dalszych ruch贸w.

### 4) Wykrywanie szafki i uchwytu :detective:
W tym etapie zadania **korzystamy z w艂a艣ciwo艣ci sterowania impedancyjnego i mo偶liwo艣ci wchodzenia w kontakt z otoczeniem**, np. aby zlokalizowa膰 uchwyt drzwi, zadano trajektori臋 kolizyjn膮 z ma艂膮 pr臋dko艣ci膮, sztywno艣ci膮 i tolerancj膮 uchybu (parametr _path_tol_).


### 5) Estymacja szeroko艣ci drzwi (_promienia szafki_)
Kiedy robot wsun膮艂 ju偶 palce chwytaka w uchwyt, wykonuje on niewielki ruch w przestrzeni kartezja艅skiej. Poci膮ga za klamk臋 w swoj膮 stron臋 na dystansie 5 cm. Ten ruch ma na celu rozpoznanie 艣rodowiska tzn. obiektu, kt贸ry podlega manipulacji. Przed uchyleniem drzwi pobierana jest pozycja uchwytu w uk艂adzie bazy. Po uchyleniu pozycja uchwytu pobierana jest ponownie. Na podstawie informacji o po艂o偶eniu i orientacji uk艂adu zwi膮zanego z klamk膮 przed i po uchyleniu estymowany jest promie艅 szafki (szeroko艣膰 drzwiczek). Do estymacji promienia wykorzystano _twierdzenie cosinus贸w_ - procedur臋 opisano w postaci schematu znajduj膮cego si臋 poni偶ej:
<p align="center"> 
  <img src="https://user-images.githubusercontent.com/80155305/150612062-620b0b57-d003-46c7-a8f9-de76f3dd6025.png"/>  
</p> 





Promie艅 wyznaczany podczas uchylania drzwi jest zgodny z rzeczywist膮 szeroko艣ci膮 szafki (zmierzon膮 w _Gazebo_).  
:warning: Przy opisanej metodzie wyznaczania promienia nale偶y zaimplementowa膰 obs艂ug臋 wyj膮tku **_ZeroDivisionError_**. W sterowaniu impedancyjnym pozycja osi膮gana jest z pewna dok艂adno艣cia. Mo偶e okaza膰 si臋, 偶e robot b臋dzie my艣la艂, 偶e chwyci艂 uchwyt, a w rzeczywisto艣ci tego nie zrobi艂. Przy uchylaniu szafki robot nie poci膮gnie za sob膮 drzwiczek - k膮t _beta_ b臋dzie r贸wny 0 stopni i w mianowniku pojawi si臋 zero.  

### 6) Generowanie trajektorii i ruch po okr臋gu
Po wyliczeniu szeroko艣ci drzwiczek generowana jest na tej podstawie trajektoria. Jest ni膮 lista _n_ punkt贸w (parametr _n_ mo偶e dobra膰 u偶ytkownik) w uk艂adzie zwi膮zanym z uchwytem szafki przed jej otwarciem. Z ka偶dym punktem zwi膮zana jest r贸wnie偶 orientacja chwytaka. Na przyk艂ad dla _n_ = 6 kolejne orientacje chwytaka to 0, 15, 30, 45, 60, 75, 90 stopni. Orientacja musi si臋 zmiania膰 - w przeciwnym razie uchyt szafki zosta艂by wyrwany.

### 7) Bezpieczne puszczenie uchwytu i powr贸t do pozycji pocz膮tkowej 
Po otworzeniu szafki do k膮ta 90 stopni wykonywane s膮 dwa ruchy w przestrzeni kartezja艅skiej. Chwytak najpierw wysuwa, a potem odsuwa si臋 od drzwiczek. Na koniec rami臋 wraca do pozycji pocz膮tkowej w trybie _JIMP_.



## Pliki 藕r贸d艂owe :card_file_box:
Finalna wersja programu steruj膮cego robotem _Velma_ w zadaniu otwierania szafki znajduje sie w  _**manipulation/scripts/project_open.py**_ tego repozytorium.  
  
## Opis implementacji 馃捇 
Podczas prac nad zadaniem do艂o偶ono wszelkich stara艅, aby program spe艂nia艂 nast臋puj膮ce wymagania: 
* **odporno艣膰 na zmiany stanu pocz膮tkowego 艣rodowiska** ( nie ma _"zahardkowowanej"_ trajektorii - jest ona generowana na podstawie pozycji uchwytu oraz szeroko艣ci drzwiczek)
* **sprawdzanie powodzenia wykonania kolejnych etap贸w zadania i reakcja na sytuacje nieoczekiwanie** - w konsoli wy艣wietlaj膮 si臋 podstawowe _logi_ pozwalaj膮ce jednoznaczenie stwierdzi膰, na jakim etapie wykonania zadania aktualnie znajduje si臋 robot.  
* **jako艣膰 kodu** - zastosowano podej艣cie obiektowe. Trzy g艂贸wne klasy to: _Initialization_, _CartSpace_ oraz _Hand_. Dodano tak偶e komentarze wyja艣niaj膮ce, do czego s艂u偶y dany fragment kodu. Nazwy funkcji dobierano tak, aby mo偶na by艂o jednoznacznie i szybko stwierdzi膰, za co dana funkcja odpowiada.  


## Instrukcja uruchomienia 馃殌   

1) W ka偶dej konsoli nale偶y wpisa膰: `source ~/mobile_ws/devel/setup.bash`
2) W osobnych konsolach wpisujemy kolejno poni偶sze komendy:  
```
roscore
roslaunch manipulation full_p2.launch
roslaunch manipulation rviz_and_objects_p2.launch
roslaunch rcprg_gazebo_utils gazebo_client.launch
rosrun manipulation project_open.py 
```
3) Przed kolejnym uruchomieniem nale偶y wyczy艣ci膰 pami臋膰 wsp贸艂dzielon膮: `rosrun velma_common reset_shm_comm.py`

## Testowanie systemu 馃И   
Przeprowadzone testy na zbudowanym 艣rodowisku wskazuj膮, 偶e system dzia艂a poprawnie. Poni偶ej za艂膮czono film poklatkowy przedstawiaj膮cy robota _Velma_ w zadaniu otwierania szafki:
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150435294-bc33f67f-0494-445d-b143-80bb06a3f1ad.gif"/>  
</p> 


### Diagramy SysML :bar_chart:
Diagramy omawiaj膮ce implementacj臋, koncepcj臋 oraz struktur臋 systemu umieszono w osobnym dokumencie :arrow_right: **[Link do diagram贸w](https://github.com/STERO-21Z/kaniuka-krasnodebski/blob/tiago/docs/projekt2_sysml.md)**













