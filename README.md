

# Intro

As part of the Historic Digital Survey ([HDS](https://cyberbuild.eng.ed.ac.uk/projects/historic-digital-survey)) research work, we have developed the Masonry Segmentation CloudCompare Plugin, which is a plugin for [CloudCompare](https://www.danielgm.net/cc/) that enables the segmentation of dense point clouds (principally from laser scanning) of masonry structures into their individual stones.

The current version of the plugin focuses on rubble masonry (and this also leads to the segmentation of mortar regions) for "straight" walls (i.e. without significant curvature). A future version shall also enable the segmentation of ashlar masonry and of walls with designed curvature (e.g. cyclindrical towers). The plugin contains two tools: one is for the automated segmentation of the point cloud into the wall's constitutive stones. The other one is to conduct this process manually either from scratch or (most commonly) to correct the errors of the automated tool (it's hard to create a perfect tool!).

# Download

Currently, the plugin is actually not made available as just a plugin .dll  that can be added to already installed versions of CloudCompare. Instead, we have packaged the plugin (actually there are two plugins; one for each tool) and CloudCompare together in a standalone Windows 10 64bit application (may work in Windows 8 as well). You only need to download the latest version available in the Releases section, unzip it and launch the CloudCompare executable it contains. The folder also contains a detailed manual explaining how to use the plugin. As per the requirements of the [GPLv2](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html) license, the source code of our plugin is freely available, upon request.

A precompiled version of CloudCompare with the plugins (x64 for Windows) is available in the Release section of this repository.

# Test Dataset

Some test datasets are available at [https://doi.org/10.7488/ds/2892](https://doi.org/10.7488/ds/2892) (University of Edinburgh's DataShare site).

# Feedback and Acknowledgement

For any feedback about the plugin, please contact Frédéric Bosché at [f.bosche@ed.ac.uk](mailto:f.bosche@ed.ac.uk?subject=Source%20code%20of%20the%20CC%20Plugin).

Furthermore, if you have used this plugin for work that is subsequently published, please acknowledge the plugin by citing one of the following publications (or ideally both):

- Valero, E., Bosché, F., Forster, A., M'Beirick, I., Wilson, L., Turmel, A. & Hyslop, E. (2019), "Development of a novel open tool for the segmentation of 3D point clouds of masonry walls", 
14th International Congress on the Deterioration and Conservation of Stone, STTONE 2020, Germany. [PDF](https://cyberbuild.eng.ed.ac.uk/sites/cyberbuild.eng.ed.ac.uk/files/attachments/freestyle-page/20200801/Valero_2020_Stone.pdf)

- Valero, E., Bosché, F. & Forster, A., (2018), "Automatic Segmentation of 3D Point Clouds of Rubble Masonry Walls, and its Application to Building Surveying, Repair and Maintenance", Automation in Construction, Vol. 96, pp. 29-39, doi:10.1016/j.autcon.2018.08.018. [PDF](https://cyberbuild.eng.ed.ac.uk/sites/cyberbuild.eng.ed.ac.uk/files/attachments/freestyle-page/20200217/segmentation%20paper_r1_0.pdf) | [web](https://doi.org/10.1016/j.autcon.2018.08.018)

- Valero, E, Bosché, F. (2020). Masonry segmentation plugin for CloudCompare, [software]. Cyberbuild Lab. The University of Edinburgh. doi:10.7488/ds/2892. [web](https://doi.org/10.7488/ds/2892)

# Contributors and our own Acknowledgements

This plugin is an output of the [Historic Digital Survey (HDS)](https://cyberbuild.eng.ed.ac.uk/projects/historic-digital-survey) research project that has been running for a number of years as a close collaboration between:

- Dr Enrique Valero and Dr Frédéric Bosché from the University of Edinburgh;
- Dr Alan Forster from Heriot-Watt University; and
- Dr Lyn Wilson, Dr Aurélie Turmel and Dr Ewan Hyslop from Historic Environment Scotland (HES)

We would particularly like to acknowledge the funding provided by HES.

We would also like to acknowledge the contributions of summer interns Ms Camille Renier and Mr Ismael M'Beirick as well as the ERASMUS programme that supported their internships.
