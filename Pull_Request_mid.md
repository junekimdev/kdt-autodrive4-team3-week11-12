# Team3 μ€κ°λ³΄κ³ 

## π© νμ μ­ν  λ΄λΉ

- κΉμΈμ€: Lead in Project Management, S/W Architecture, ML on AWS
- κΉλ€ν: Lead in Data Acquistion, Labeling, and Augmentation
- κΉλν: Lead in Object Detection Logic
- μ΄μΈμ: Lead in Xycar Control Logic

## νμ΅ μ½λ λ° μ£Όν μ½λ(git repository νμ)

- νμ΅μ½λ: <https://github.com/junekimdev/yolov3-pytorch>
  > This is a fork from <https://github.com/2damin/yolov3-pytorch>
- μ£Όνμ½λ: <https://github.com/junekimdev/kdt-autodrive4-team3-week11-12>

## π©βπ» νμ΅ κ³Όμ  κ³΅μ 

### loss κ·Έλν, accuaracy κ·Έλν

- Evaluation graphs

  ![eval](tensorboard-graphs_mid/eval_graph.png)

- Loss graphs

  - box_loss

    ![box_loss](tensorboard-graphs_mid/box_loss.png)

  - cls_loss

    ![cls_loss](tensorboard-graphs_mid/cls_loss.png)

  - example

    ![example](tensorboard-graphs_mid/example.png)

  - lr

    ![lr](tensorboard-graphs_mid/lr.png)

  - obj_loss

    ![obj_loss](tensorboard-graphs_mid/obj_loss.png)

  - total_loss

    ![total_loss](tensorboard-graphs_mid/total_loss.png)

### iteration λλ epoch

- 500 epoch

## λ°μ΄ν°

### (ν΄λμ€λ³) νμ΅ λ°μ΄ν° μ

> total number of images: 3393

| class  | left | right | stop | crosswalk | utrun | traffic_light | None |
| :----: | :--: | :---: | :--: | :-------: | :---: | :-----------: | :--: |
| number | 1468 | 1393  | 759  |    874    |  61   |      966      | 451  |

### (ν΄λμ€λ³) νκ° λ°μ΄ν° μ

> total number of images: 395

| class  | left | right | stop | crosswalk | utrun | traffic_light | None |
| :----: | :--: | :---: | :--: | :-------: | :---: | :-----------: | :--: |
| number | 433  |  136  |  57  |    27     |   0   |      36       |  27  |

### λ°μ΄ν° μμ§ λ°©λ² <!-- μμ: νΈλν° μΉ΄λ©λΌ, μμ΄μΉ΄ μΉ΄λ©λΌ λ± -->

μμ΄μΉ΄ μΉ΄λ©λΌ

### μ¬μ©ν μ΄κ·Έλ©νμ΄μ

Pytorch defaul augmentation & κ°μ¬λ imgaug

## νμ΅ λͺ¨λΈ ν¬ν μ¬λΆ

- Pytorch to darknet weight: DONE
- Darknet weight to ONNX: DONE
- ONNX to TensorRT: Not yet
- Running inference: Not yet

## β μ΄λ €μ΄ λΆλΆ

1. Evaluation graphλ₯Ό λ³΄λ, right signμ recallμ κ΄μ°?μΌλ precisionμ΄ 0.2 μ λλ‘ λ§€μ° λ?μ΅λλ€.
   - Right signμ precisionμ λμΌ λ°©λ²μ μ°Ύκ³  μμ΅λλ€.

## β μ λκ³  μλ λΆλΆ

1. μμ΄μΉ΄λ₯Ό ν΅νμ¬ μμμ νλνκ³ , νμ μ μ²΄κ° labelingμ λΉ λ₯΄κ² μννμ¬, μλΉλμ datasetμ νλ³΄νμμ΅λλ€.
2. AWSμ datasetμ uploadνμκ³ , κ°μ¬λμ yolov3-tinyλ₯Ό κΈ°λ°μΌλ‘ μ½κ°μ trouble-shootingμ ν΅ν΄ νμ΅νμμ΅λλ€.
3. 500νμ epochμ νμ΅νμκ³ , ONNXλ‘ κ·Έ κ²°κ³Όλ₯Ό portingνμμ΅λλ€.
4. μ§λ projectμμ μ¬μ©νλ s/w architectureλ₯Ό κΈ°λ°μΌλ‘ lane detectionκ³Ό control logicμ μμ± μ€μ μμ΅λλ€.
