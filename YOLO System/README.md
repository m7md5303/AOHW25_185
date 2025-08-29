<h1>Brief Description for the sub-directory:</h1>
<p>The used YOLO model can be found <span><a href="https://github.com/sefaburakokcu/quantized-yolov5">here</a></span>
, some small edits were done for configuring the model with the desired dataset. The ready model is included as "EA_lpyolo.rar
<br>
Additionally, the configuration file for the used <span><a href="https://www.kaggle.com/datasets/seyeon040768/car-detection-dataset">dataset</a></span> is provided as cars.yaml
<br>
The uploaded Jupyter Notebooks are for the rest of the flow:</p>
<ul>
  <li>Training</li>
  <li>Exporting to QONNX</li>
  <li>Generating the IP with FINN</li>
  <li>Testing the intermediate generated ONNX files</li>
  <li>The Post-processing Verilog Module</li>
  <li>Simple testbench for the whole system whose architecture is at Sys bd.png</li>
</ul>
