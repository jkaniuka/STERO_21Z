# Projekt 2 - diagramy SysML 📊

 ### :arrow_forward: Diagram wymagań (req):   
 **Komentarz:** Środkowy element diagramu jest **wymaganiem nadrzędnym**, a wokół niego znajdują się **wymagania zagnieżdżone**. 
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150440233-2c1bc980-bf90-4a01-8805-e9dfd0f6b2eb.png" width="500" height="500"/>  
</p> 



### :arrow_forward: Diagram aktywności (act): 
 **Komentarz:** Początkowo wykonaliśmy _diagram sekwencji (seq)_ dla całego systemu. Okazało się jednak, że jest on zbytnio rozbudowany tzn. zbyt "długi" z przytłaczającą liczbą strzałek i napisów. Postanowiliśmy zastąpić go diagramem behawioralnym w postaci diagramu aktywności (act) i wykonać **mapowanie** każdej z aktywności na poddiagram diagramu sekwencji. Dekompozycja połączona z mapowaniem znacząco ułatwia zrozumienie struktury zaimplementowanego programu.
   
Opis każdej aktywności kończy się adnotacją w nawiasie kwadratowym - jest to odniesienie do odpowiedniego diagramu sekwencji.
 
![image](https://user-images.githubusercontent.com/80155305/150639123-63d80d9f-2a21-4619-98d7-297dd41bfd41.png)

 
### :arrow_forward: Diagramy sekwencji (seq) - mapowane z diagramem aktywności:
 **Komentarz:** Na górze diagramu aktywności znajdują się **bloki** - w naszym przypadku są to obiekty klas, na których wywoływane są kolejne metody. Z tego względu nie dodawaliśmy np. bloku _Gazebo_, ponieważ wtedy grupa obiektów byłaby niejednorodna pod względem typu bloku.  
 
 ### :arrow_right: Inicjalizacja:
 <p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150639353-870e8f1a-7115-4f90-ba30-a2c9eb4b0fe2.png" />  
</p> 

### :arrow_right: Zbliżenie się do szafki, zmiana impedancji:  
Funkcja move_wrt_frame() oznacza _move with respect to frame()_ - wykonuje ruch do pozycji zadanej w wybranym układzie.

  <p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150639160-12142728-9f08-4df5-92c7-dec2302d0234.png" /> 
  </p> 
  
### :arrow_right: Estymacja szerokości drzwi, generacja trajektorii, wykonanie ruchu:  

   <p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150639175-f6b8deea-12b4-4301-aab2-62ad8274c910.png" />
   </p> 
   
### :arrow_right: Odsunięcie się od szafki, powrót do pozycji początkowej:

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150639199-a70e66b1-de79-49e4-bb1a-656eba4aa034.png" />  
</p> 






### :arrow_forward: Diagram definiowania bloków (bdd):  
**Komentarz:**
*  _Octomapa_ jest blokiem **agregowanym** przez _Planner_, ponieważ octomapa jest bytem, który może istnieć bez plannera. Mapa środowiska może być wykorzystana np. przez innego robota.
*  _Planner_ jest blokiem **agregowanym** przez _Velma Interface_, ponieważ zadanie może zostać wykonane bez użycia Plannera i przy korzystaniu tylko z ruchów w trybie CIMP
*  _Hand_ i _Cart_ są blokami **komponowanymi** przez _Velma Interface_, ponieważ np. gripper jest integralną częścią robota Velma.
*  _Velma Interface_ jest blokiem **komponowanym** przez _my_node_, ponieważ nasz węzeł bez _Velma Interface_ nie będzie w stanie obsłużyć zadania otwierania szafki.
  



<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/150640303-e2f7ae8e-8ca8-4e74-bbcb-50af4c38555f.png" width="500" height="500"/>  
</p> 
