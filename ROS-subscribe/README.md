<h1> subject </h1>
하나의 node에서 2개 이상의 토픽 subscribe하기 </br>
노드는 다음과 같이 구성한다. </br></br>
- 1번 node : 1과 10 사이의 랜덤한 값을 A로 publish </br>
- 2번 node : 11과 20 사이의 랜덤한 값을 B로 publish </br>
- 3번 node : A와 B를 subscribe한 후, A+B를 연산하여 C로 publish </br>
- 4번 node : C subscribe 누적 평균 출력 </br>
</br>
</br>

<h2> 1. 구성 </h2>

- 1번 node </br>
별첨 소스코드 중 node1.cpp 에 구현. </br>
rand 함수를 사용하여 1~10 까지의 숫자를 랜덤으로 선택. </br>
”topicA“ 버스를 통해 데이터를 전송하는 publisher 구현 </br>
1초 마다 한번 씩 랜덤으로 선택한 숫자를 전송한다. 

- 2번 node </br>
별첨 소스코드 중 node2.cpp 에 구현. </br>
rand 함수를 사용하여 11~20 까지의 숫자를 랜덤으로 선택. </br>
”topicB“ 버스를 통해 데이터를 전송하는 publisher 구현 </br>
1초 마다 한번 씩 랜덤으로 선택한 숫자를 전송한다.  </br>

- 3번 node </br>
별첨 소스코드 중 node3.cpp 에 구현. </br>
“topicA“ 와 “topicB“ 버스를 subscribe 하여 안에 담긴 데이터를 읽는다. </br>
각 데이터의 평균을 구하고, ”topicC“ 버스를 통해 데이터를 전송하는 publisher 구현 </br>
1초 마다 한번 씩 값을 전송한다.

- 4번 node </br>
별첨 소스코드 중 node4.cpp 에 구현. </br>
“topicC“ 버스를 subscribe 하여 안에 담긴 데이터를 읽는다. </br>
 1초 마다 한번 씩 읽은 데이터들의 총 평균값을 계산하여 출력한다. </br>

- CMakeLists.txt </br>
CMakeLists 파일 하단부에 구현한 node의 이름과 해당 node를 구현한 소스 파일의 위치를 명시해주었다. </br>
add_executable 과 target_link_libraries 함수를 사용하여 총 4개의 node를 컴파일 목록에 추가하였다. </br>


<h3> 2. 실행결과 </h3>

프로젝트를 실행하기 위해 총 5개의 터미널을 사용하였다. </br>
roscore 를 먼저 실행한 후, 4개의 node를 순차적으로 실행하였다.</br>

- roscore 실행
<img align="center" src="https://user-images.githubusercontent.com/54891281/201509813-0f0d0bf6-de79-4f6b-8694-ca347a7bed1b.png" width="600"/>

|1번 node ( node 1) 실행|2번 node ( node 2) 실행|
|------|------|
|<img align="left" src="https://user-images.githubusercontent.com/54891281/201509815-caa80492-9e76-4d15-a89d-2e1114a4272e.png" width="300"/>|<img align="right" src="https://user-images.githubusercontent.com/54891281/201509818-8c0f09dd-2dc8-4cb7-9e22-6416e3f40f7b.png" width="300"/>|



|3번 node ( node 3) 실행|4번 node ( node 4) 실행|
|------|------|
|<img align="left" src="https://user-images.githubusercontent.com/54891281/201509820-736a35fb-37d2-4d78-8b8b-f32303a762d7.png" width="300"/> | <img align="right" src="https://user-images.githubusercontent.com/54891281/201509811-92223203-396b-4f0f-9eb7-469678981b16.png" width="300"/>|


4개의 node를 모두 동시에 실행하는 것이 아니고 각 개별로 실행하다보니 조금씩 오차가 발생하였다.</br> 
3번 node는 2번 node에서 생성한 랜덤 값 11을 수신하지 못하였다.</br>
4번 노드는 실행 초기 값을 입력받지 못하여 초기값으로 설정한 0이 그대로 출력되어 누적평균에 오차를 주었다. </br>
각 node별로 실행시간의 차이에 따라 초기에는 에러가 발생했지만, 1,2,3번 node는 실행 약 2초 후 안정적으로 동작하였으며, </br>
4번 node는 약 3초 후 안정적인 모습을 보였다.

