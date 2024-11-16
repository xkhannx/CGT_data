# Cold-Gas Thruster

Cold-gas thruster (CGT) is intended as a backpack-wearable device, for purposes of arresting backward falls. This project is developed by CREATe Lab at Vanderbilt University. Please refer to the published papers on this project:
Journal papers:
- A. Baimyshev, M. Finn-Henry, and M. Goldfarb, "A supervisory controller intended to arrest dynamic falls with a wearable cold-gas thruster," Wearable Technologies, 4, E23, 2023. doi:10.1017/wtc.2023.18.
- A. Baimyshev, M. Finn-Henry, and M. Goldfarb, "Feasibility of a Wearable Cold-Gas Thruster for Fall Prevention," J. Dyn. Sys., Meas., Control, vol. 144, no. 8, p. 084501, 2022, doi: 10.1115/1.4054529.

Conference proceedings:
- M. Finn-Henry, J. L. Brenes, A. Baimyshev and M. Goldfarb, "Preliminary Evaluation of a Wearable Thruster for Arresting Backwards Falls," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 12604-12609, doi: 10.1109/ICRA48891.2023.10160518.
- M. Finn-Henry, A. Baimyshev, and M. Goldfarb, "Feasibility Study of a Fall Prevention Cold Gas Thruster," 2020 8th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob), 2020, pp. 611-616, doi: 10.1109/BioRob49111.2020.9224425.


This repository contains the experiment and simulation data from the Cold-Gas Thruster project and the code used to process it. There are five folders here:

1. Board -- contains the data collected from the embedded system that controls the CGT device. The main MATLAB script here is named "processBoardData.m" that plots the data from the board. The file "boardMocapRMS.m" calculates the RMS error between the board data and the MoCap data. The rest are helper functions. 
2. MoCap -- contains the motion capture data collected during experiments. There are two main scripts that process and plot the data.
3. OpenSim -- contains a Visual Studio C++ project that creates an OpenSim model of a two-link pendulum. Please refer to the OpenSim documentation for assistance: https://simtk.org/api_docs/opensim/api_docs32/ 
4. OpenSimData -- contains the data from simulations of a two-link pendulum human model. The "plotSim.m" script processes and plots all data.
5. Videos -- contains frames from videos of the experiments and a video of an elderly person falling. Scripts "alignVideo.m" and "alignVideoWithout.m" were used to superimpose the data from the videos and the motion capture data to synchronise them and find the time of the release of the block and the activation time of CGT. The "elderly3d.m" script processes the data from a video of an elderly person falling.

For any questions please contact me at almaskhan.baimyshev@vanderbilt.edu.
