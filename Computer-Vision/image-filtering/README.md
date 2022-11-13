

<h1> 1. 히스토그램 평탄화 </h1>

|기존이미지|
|-----------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201514086-171911dc-84f4-434d-be65-61ccd7915eca.png" width="400"/>|

|이미지 histogram|
|-----------|
|이미지의 red 성분을 추출하고, 히스토그램으로 표현|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201514088-0889bd8a-838a-4966-956e-2bfb2cdc2b7b.png" width="400"/>|


|평탄화 적용|
|-----------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201514089-ff8af2ee-c7ef-42e9-8830-14a5e2f7f546.png" width="400"/>|


|평탄화 적용 color|
|----------------|
|<img align="center" src="https://user-images.githubusercontent.com/54891281/201514092-0ed15650-a7a7-4529-9176-f24d5726b5d9.png" width="400"/>|


<h1> 2. 공간 도메인 필터링 </h1>

<img align="center" src="https://user-images.githubusercontent.com/54891281/201514239-93fbf989-4cb6-49c0-9cdc-8e3c0f9c0a28.PNG" width="400"/>
<img align="center" src="https://user-images.githubusercontent.com/54891281/201514454-9fa73175-5a7c-44fc-8c19-bbeaa8774e9f.PNG" width="400"/>


diameter, sigmacolor, sigmaspace 는 Bilateral Filter 의 파라미터로 사용된다 </br>
기본적으로 전체적인 노이즈를 감소시키는 효과가 있으며, 이미지의 경계선(엣지)는 어느 정도 보존시키는 장점이 있다. </br>
sigmaspace를 30->80 으로 증가하였을 때 큰 변화는 보이지 않는다.</br>
반면 sigmacolor를 30->50 으로 변화를 주었을 때 전체적인 노이즈가 감소하는 것이 보이며, diameter는 약간의 변화에도 이미지는 큰 변화가 보인다</br>


<h1> 3. 주파수 도메인 필터링 </h1>

<img align="center" src="https://user-images.githubusercontent.com/54891281/201514258-570d1c1e-7454-4dcd-9268-0f32c29490fb.png" width="400"/></br>
반지름 160의 원을 mask 로 이용하였으며, High pass로 원 바깥의 이미지를 통과시켰다 </br>

<img align="center" src="https://user-images.githubusercontent.com/54891281/201514259-8121f669-f047-41d2-8644-dc01c69b5fbf.png" width="400"/></br>
반지름 160의 원을 mask 로 이용하였으며, Low pass로 원 안쪽의 이미지를 통과시켰다 </br>


<h1> 4. 모폴로지 필터 </h1>

<img align="center" src="https://user-images.githubusercontent.com/54891281/201514261-edc629ee-d2a5-457f-bf16-bf82cf834038.png" width="400"/></br>
Erosion, Dilation, Opening, Closing 각각의 횟수를 입력받아서 반복 수행한 결과이다 </br>


