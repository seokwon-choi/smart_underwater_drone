## AI기반 수중드론을 활용한 선박 청소 시스템
![one](https://user-images.githubusercontent.com/85105917/132091960-72a2cd6a-821c-4bb1-888a-c3ed10f1050f.JPG)

##### 1.담당 업무 : Keil uVision5 환경에서 수중드론 제어, CNN을 활용한 이미지 분석
##### 2. 개발 기획 (WBS를 작성해 프로젝트 일정을 계획)
![image](https://user-images.githubusercontent.com/85105917/132093714-218a252b-e344-4698-9dc2-c2d3a2e0de5d.png)
##### 2-1. 울산항만청 관계자와의 인터뷰를 바탕으로 서비스 구성도 설계 : 현재 시스템 분석과 요구사항 파악
1. 현재 시스템은 200m이상 선체 청소 시 잠수부 6~7명이 투입되어 1주일 정도 작업한다는 답변을 받음
2. 현대 상선에서 수중드론을 이용하여 선체 청소를 하는 사례가 있지만 기술이 완벽하지 않다는답변을 받음
3. 잠수부를 도입하여 청소하는 것 보다 비용 절감이 30~40%정도 라고  예상한다는 답변을 받음
4. 수중드론으로 촬영한 영상을 분석해 주어 청소 여부를 알려준다면 더 유용할 것이라는 답변을 받음
![image](https://user-images.githubusercontent.com/85105917/132093817-8ef3eb30-1949-4175-a90c-63767bfc6993.png)
![image](https://user-images.githubusercontent.com/85105917/132093898-92dc1d6b-4b54-427e-9623-d5650c156eae.png)
##### 2-2 서비스 흐름도를 바탕으로 메뉴 구성도 설계
![image](https://user-images.githubusercontent.com/85105917/132094306-08be816c-f5d9-4596-bfff-0ce828771c6f.png)
##### 2-3 기능 설계도를 설계하여 시스템 분석 및 설계
![image](https://user-images.githubusercontent.com/85105917/132094379-4ab60644-0866-4fd8-a0ba-5b323df3d151.png)
![image](https://user-images.githubusercontent.com/85105917/132094395-7eb02b6c-8468-411b-b597-f29e0b59d234.png)
##### 3. 프로젝트 내용 : 수중드론 개발, AI 이미지 분석 프로그램, 청소 요청 및 이미지 분석 결과 전달 APP(필자가 개발에 참여한 부분이 아니므로 자세한 설명 생략)
##### 3-1. 수중드론 개발
![image](https://user-images.githubusercontent.com/85105917/132095672-94b08a42-1dff-43eb-a708-44e1488d087a.png)
1. 조종기의 신호에 따라 수중드론 동작 제어를 설정한다. ex) 속도 조절, 방향 제어 등
2. BLDC모터는 ESC를 사용하기 때문에 Calibration을 해주어야한다.
3. Calibration 방법은 최대 DUTY와 최소 DUTY를 입력시켜 진행한다. 본 프로젝트는 60%, 20%를 부여하였다.
4. 60%와 20%의 중간인 40%가 되면 BLDC모터는 정지한다. 40%이하이면 역방향으로 40%에서 멀어질수록 속도가 빨라진다.
5. 반대로 40%보다 크면 정방향이고 속도 조절은 역방향과 같다.
6. 본 프로젝트는 추진 모터 2개로 직진, 후진, 좌회전, 우회전을 제어하였고 4개의 모터로 상승, 하강, 정지비행을 제어하였다.







