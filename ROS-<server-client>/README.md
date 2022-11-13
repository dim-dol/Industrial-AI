<h1> subject </h1>
1초마다 랜덤하게 n개의 반지름을 생성하여 각각의 반지름을 계산하여 넓이를 계산해주는 서비스 구현 </br>
</br>
- 1초마다 random으로 n개의 반지름 생성 </br>
- int array를 input, float array를 output 으로 하는 srv 생성 </br>
- input array 각각의 값을 반지름으로 하는 원의 넓이를 계산해 output array로 return </br>
- Header header와 int array(반지름), float array(넓이)를 포함하는 custom msg로 publish </br>
- Rostopic echo [topic name]으로 결과확인 </br>
</br>
</br>

<h2> 1. 구성 </h2>

- testsrv_client </br>
별첨 소스코드 중 testsrv_client.cpp 에 구현.</br>
반지름의 개수를 사용자로부터 입력받는다.</br>
1초마다 입력받은 수만큼의 반지름 값을 랜덤으로 생성한다</br>
server를 호출하여 server 가 반지름 값을 사용할 수 있다.</br>
1초마다 반지름, 넓이 값 등을 publish 한다 </br>

- testsrv_server</br>
별첨 소스코드 중 testsrv_server.cpp 에 구현.</br>
service 요청이 오면 반지름 값을 읽어서 넓이를 계산한다.</br>
원주율은 math.h 라이브러리의 M_PI를 사용하였다.</br>

- testsrv.srv </br>
별첨 소스코드 중 testsrv.srv 에 구현.</br>
input 과 output 이 구분되어있다.</br>
반지름 값 배열, 넓이 값 배열, 매초 당 생성한 반지름 개수, 순차번호</br>
위 4개의 정보를 정의한다.</br>

- testmsg.msg</br>
별첨 소스코드 중 testmsg.msg 에 구현.</br>
반지름 값 배열과 넓이 값 배열을 정의한다.</br>
publisher 가 사용할 데이터이다.</br>


<h3> 2. 실행결과 </h3>

프로젝트를 실행하기 위해 총 4개의 터미널을 사용하였다. </br>
roscore 실행 후, 2개의 node를 server, client 순으로 실행하고, rostopic echo를 실행하였다.</br>

- roscore 실행
<img align="center" src="https://user-images.githubusercontent.com/54891281/201510603-8b737a2d-4c43-4f56-ba52-acf027764c22.png" width="600"/>

|testmsg.msg|
|-----------|
|int64[] radius_array</br>float64[] area_array|
|빌드 시 testmsg.h 헤더파일을 생성하며, 내부 정보들은 publish된 topic의 항목들이다.|


|testsrv.srv|
|-----------|
|- input -</br>int64[] radius_array</br>int64 num</br>int64 flag</br>- output -</br>float64[] area_array|
|빌드 시 testsrv.h 헤더파일을 생성하며, 내부 정보들은 server 와 client 간 사용하는 정보이다.</br>server 기준으로 client 로부터 input 의 항목들을 받고, output 의 항목을 출력한다.|


|testsrv_server (server) 실행|
|-----------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201510604-820ca929-dd73-4be7-9007-fc49e42cb56f.png" width="600"/>|


|testsrv_client (client) 실행|
|-----------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201510601-cd5630ae-30bc-4872-9f02-dfe9fe6993d1.png" width="600"/>|


 server를 먼저 실행시켜서 호출대기를 시켜놓는다. client 실행 중 server를 호출하는 기능이 수행될 때 server 가 실행되어있지 않으면 에러가 발생한다.</br>

 client를 실행할 때에는 1초당 생성할 반지름의 개수를 지정하여 입력한다. 입력이 없을시, 기본 값은 5개로 설정하였다. </br>
 랜덤 값의 반지름을 생성하고 serviceclient 가 사용할 수 있도록 srv의 radius_array에 저장한다. server가 넓이 계산이 끝나면 srv의 반지름과 넓이 값을 publisher 가 publish 할 수 있도록 저장하고 publish를 수행한다 </br>

 server는 client가 호출하면 srv의 radius_array에서 반지름 값을 가져온다. </br>
 그리고 반지름 값과 원주율을 이용하여 원의 넓이를 구한다. 넓이 값은 srv의 area_array 에 저장한다.</br>

|rostopic echo 실행|
|-----------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201510602-a853a823-66c8-4452-9ad2-39528b3cfbfc.png" width="600"/>|

rostopic echo를 사용하여 결과를 확인하였다. 이 명령어는 publish 된 topic custommsg의 내용을 출력해준다



