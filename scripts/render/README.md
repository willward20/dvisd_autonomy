This directory contains code to render the car's lidar for lesson 6. 
Because the rasperry pi is so small, it cannot render the code itself.
Instead, it broadcasts the data over the internet, and this script reads the
raw data and renders it. Therefore, you run this code on your laptop, and
at the same time, run either script in lesson 6. 

TLDR: Run this code on your laptop only, not on the rasberry pi