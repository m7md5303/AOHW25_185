<h1>Accelerated YOLO-Based Computer Vision System</h1>
<br>
<p>The trend towards full autonomous cars has increased widely recently. Hence, the purpose of this project is to provide pure-hardware systems allowing for
autonomous driving or any self-moving robot. Our work two main components are YOLO-based object detector for detecting cars and Sobel-based edge detector for detecting lanes.
Both the systems are free from software with just employing hardware blocks. Reaching FPS of 67 for the YOLO block and about 800 FPS for the Sobel block, one would employ
these blocks in his system. This project is pretty suitable for edge-computing devices with performing heavy-computational tasks on the Programmable Logic (PL)
of FPGAs.</p>
<h1>Team Members</h1>
<ul>
  <li>Mohamed Khaled, Email:<a href="mailto:m7md5303@gmail.com">m7md5303@gmail.com</a></li>
  <li>Saber Mahmoud, Email:<a href="mailto:saber.mahmoud702@eng-st.cu.edu.eg">saber.mahmoud702@eng-st.cu.edu.eg</a></li>
  <li>Saif Gebril, Email:<a href="mailto:saifnasser144@gmail.com">saifnasser144@gmail.com </a></li>
  <li>Saleh Sharouk, Email:<a href="mailto:salehhesham529@gmail.com">salehhesham529@gmail.com</a></li>
  <li>Seif M. Megahed, Email:<a href="mailto:seifmegahed13@gmail.com">seifmegahed13@gmail.com</a></li>
</ul>
<h3>Supervisor:</h3>
<ul>
  <li>Dr.Ibrahim M. I. Qamar, Email:<a href="ibrqamar@gmail.com">ibrqamar@gmail.com</a></li>
</ul>
<h1>Boards Used:</h1>
<ul>
  <li>YOLO Block was deployed on: ZCU102 and PYNQ-Z2 (Applied different levels of set parallelism)</li>
  <li>Sobel Block was deployed on: PYNQ-Z2</li>
</ul>
<h1>Software Needed:</h1>
<ul>
  <li>Vivado 2022.2</li>
  <li>Vitis IDE 2022.2</li>
  <li>FINN Framework</li>
  <li>PYNQ image v3.0.1</li>
</ul>
<h1>Repository Architecture:</h1>
<p>
  YOLO System folder: This directory includes all the source files for implementing the YOLO Hardware System from scratch (Training Step)
  <br>
  It has two sub-directories ZCU102_HW and PYNQ-Z2_HW, where both includes either Vitis files in case of ZCU102 or the Jupyter Notebook required files in the case of PYNQ-Z2.
</p>
<p>
  Lane System folder: This directory includes all the design file for the Lane block
  <br>
  It has one sub-directory: PYNQ-Z2_HW, which includes the required files for the Jupyter Notebook operation on this very design.
</p>
