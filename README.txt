README file for

QRONOS-QoC-Stochastic

Stochastic Quality-of-Control evaluation for real-time control systems with varying timing

COPYRIGHT AND LICENSE:
- Main folder:
  (C) Maximilian Gaukler 2017-2018
  GPL3, see LICENSE.txt
- Libraries: see the license information in the subfolders:
  git-archive-all/git-archive-all.sh
  matlab2tikz/LICENSE
  matlab-xunit/license.txt
  MBeautifier/LICENSE

BACKGROUND / THEORY:
See the following publication for the theoretical background of this implementation and the definition of most variable names:

Maximilian Gaukler, Andreas Michalka, Peter Ulbrich, and Tobias Klaus.
2018. A New Perspective on Quality Evaluation for Control Systems with Stochastic Timing.
In HSCC ’18: 21st International Conference on Hybrid Systems: Computation and Control (part of CPS Week), April 11–13, 2018, Porto, Portugal.
ACM, New York, NY, USA, 10 pages.
https://doi.org/10.1145/3178126.3178134



REQUIREMENTS:
The software *should* work in the following environment. A more strict specification can be found in the next paragraph.
- Standard PC (something like 8 GB RAM and 200 GB HDD is more than enough).
- Matlab R2015a or R2017a (or maybe newer) with Simulink, Control System Toolbox, DSP Toolbox, Signal Processing Toolbox.
  (Matlab R2015a was used for development. Tests were also run on R2017a. For development, the Simulink models have to be edited with R2015a so that backwards compatibility is not broken.)
- Windows 7 or 10 (Linux may work as well)
- Matlab must be correctly set up so that the Simulink Rapid Accelerator Mode works. Installing a compiler may be required for R2015a, and should not be required for 2017a.
More information is available in "Supported and Compatible Compilers – Release 2015a":   
  https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/support/sysreq/files/SystemRequirements-Release2015a_SupportedCompilers.pdf

  
TESTED CONFIGURATION:
To rule out hidden dependencies, the software was tested with this configuration and instructions:
- [64bit Virtual machine with 8GB RAM and 2TB (way more than enough) virtual HDD using VirtualBox on 64-bit Intel i7-4790 on Win7 x64]
- Windows 10 Enterprise Version 1709 Build 16299.129, 64bit German
  - installed from SW_DVD5_WIN_ENT_10_1703_64BIT_German_MLF_X21-36490.iso
  - all updates installed as of 2017-01-08 (reboot and search for updates again until no updates show up anymore)
  - [all privacy settings "disabled"]
  - [for the virtual machine, install VirtualBox guest additions]
- Install Matlab R2017a with features:
	 - MATLAB
	 - Simulink
	 - Control System Toolbox
	 - DSP System Toolbox
	 - Signal Processing Toolbox
- Please note that MATLAB R2017a has a bug which makes it crash if ("internal error") it is idle (doing noting) for too long, when using a network license with idle timeout. This is completely unrelated to our software. If you are affected, install the patch from http://www.mathworks.com/support/bugreports/1554513 (or add "pause(Inf)" to the end of the script as a workaround). For testing, this patch was applied.
- add non-admin user
- start MATLAB as this user
- copy the files to a writable location on the local disk (C:\User\somebody\Documents\MATLAB\something), preferably without any special characters or whitespace in the path.
- in MATLAB, change to the folder, open run_all.m and run it
- wait for about two days [may be about three times slower in a virtual machine, for unknown reasons] until the results are ready.
- The very first figure window that opens is Figure 3 from the HSCC2018 publication.
- press Cancel on any Windows Defender Firewall dialogs that appear

INSTALL:
- from a ZIP file: unpack the ZIP file containing everything to some directory on the local disk
- from a git repository: Recursively clone the git repository (git clone --recursive <url>)

RUNNING AND REPRODUCING FIGURES:
- Open Matlab, change to the directory containing run_all.m, start run_all.m
- All unit-tests and examples are run sequentially. This may take a few days (!) for all figures, and about five hours until the first figure.
- If Windows Firewall dialog boxes appear, dismiss them with "Abort", "Cancel" or similar. No network access (except to localhost) is necessary, this dialog box is merely a side effect of using Simulink's Rapid Accelerator Mode.
- The computed figures are displayed; the output is saved to multiple subfolders plot-archive/<date>-<git revision>-*/, one per example.
- The paper contains exactly one figure which is produced by MATLAB: The numeric example (Figure 3).
  This is the very first MATLAB figure window opened by run_all.m (or example_pendulum_timevarying.m).
  To produce the exact style from the paper, the data is exported using matlab2tikz and then imported into a custom pgf-plot (which is not included here).

CUSTOMIZING:
For getting started, take a look at example_integrator.m, which is self-contained and does not reference external parameters.
The other examples are designed to be as modular as possible; parameters for the pendulum examples are sourced from param_pendulum_*.m.
To modify their parameters, change param_pendulum_* and re-run example_pendulum_....


CODE OVERVIEW:
Besides a dozen helper functions, this package contains:
- stochasticJumplinearSystem: stochastic discretization for linear impulsive systems
- controlledSystem: QoC computations for a closed loop consisting of continuous-time plant, discrete-time controller and sample-hold units (representing analog-to-digital and vice versa) with varying timing. Builds upon stochasticJumplinearSystem.
- verification_compare_timevarying_qoc: Compare the output of controlledSystem to a Simulink simulation, with a slightly limited range of parameters.
- examples and unit-tests which are called from run_all.m (see there for an overview, see the individual files example_*.m for more details).

QUESTIONS:
(Outside of the repeatability evaluation, which is anonymous and therefore does not permit direct contact.)
Feel free to contact me via max.gaukler@fau.de and tell me if you find this useful (or not). Feedback and collaboration is appreciated.
