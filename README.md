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
  <li>YOLO Block was deployed on: ZCU102 and PYNQ-Z2 (Applied different levels of set parallelism and clock frequency)</li>
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
<h1>Steps to build:</h1>
<h2>YOLO System:</h2>
<h3>ZCU102:</h3>
<p>Create new Vitis platform using the provided XSA file (yolo_zcu102.xsa) <span><a href="https://docs.amd.com/r/en-US/ug1400-vitis-embedded/Creating-a-Platform-Component-from-XSA">doc</a></span> <br>
Add the main.c file to the project sources and the provided testing image header file as well.
<br>
Create new debugging configuration (GDB) <span><a href="https://xilinx.github.io/Embedded-Design-Tutorials/docs/2022.1/build/html/docs/Introduction/Zynq7000-EDT/3-debugging-vitis.html">doc</a></span>
<br>
After Clicking run, open Vivado for viewing the output waveform on the ILA</p>
<h3>PYNQ-Z2:</h3>
<p>Setup the PYNQ image for PYNQ-Z2 <span><a href="https://pynq.readthedocs.io/en/v2.3/getting_started/pynq_z2_setup.html">doc</a></span>
<br>
Browse to the board address, upload the provided files to their specified locations in the notebook, then run all cells for watching the output
<br>
Provided testing image can be found with name img2.txt</p>
<ul>
  <li><h5>You should note that the last cell is to be run twice due to issues related to the AXI DMA IP on Jupyter Notebooks.</h5></li>
</ul>
<h2>Lane System:</h2>
<h3>PYNQ-Z2:</h3>
<p>Setup the PYNQ image for PYNQ-Z2 <span><a href="https://pynq.readthedocs.io/en/v2.3/getting_started/pynq_z2_setup.html">doc</a></span>
<br>
Browse to the board address, upload the provided files to their specified locations in the notebook, then run all cells for watching the output
<br>
Provided testing image can be found with name test1_crop.txt</p>
<ul>
  <li><h5>You should note that the last cell is to be run twice due to issues related to the AXI DMA IP on Jupyter Notebooks.</h5></li>
</ul>
