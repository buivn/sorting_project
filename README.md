# AdjustedFiles_GPGGPD
In my CV project, I used package GPD to generate Grasp Pose for my Aubo I5 robot to grasp standing objects. To work, there are some code files in the package have adjusted:\
- camera.h, camera.cpp, grasp.h, grasp.cpp belong to the GPG package, which was added functions which can filter more features of the input point cloud data and sent back the new data for GPD package.\
- Moreover, three files: classify_candidates_bd2.cpp, classify_candidates_bd3.cpp and twoCamStatic_bd1.cpp were written to handle with new requirement in classifying and selecting new object set. The files would run in GPD package.


