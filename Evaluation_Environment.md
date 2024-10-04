# Evaluation Process

When you request an evaluation, we will:
1. clone the specified branch from your private repository,
2. overwrite the set of unmodifiable files from the start-kit (you should not modify them anyway),
3. build the docker environment and compile your code in it,
4. run the evaluation process to compute a score for the leaderboard.

## Unmodified Files

In **any track** of the competition, don't modify or interfere with any start kit functionalities, including those in the following files:
```
src/ActionModel.cpp, src/Evaluation.cpp, src/Logger.cpp, src/States.cpp,src/driver.cpp,
src/CompetitionSystem.cpp, src/Grid.cpp, src/common.cpp, src/TaskManager.cpp, 
inc/ActionModel.h, inc/Evaluation.h, inc/Logger.h, inc/SharedEnv.h, inc/Tasks.h, inc/CompetitionSystem.h, inc/Grid.h,
inc/States.h, inc/common.h, inc/TaskManager.h,
default_planner/Memory.h, default_planner/heap.h, default_planner/pibt.cpp, default_planner/search_node.h, 
default_planner/planner.h, default_planner/search.cpp, default_planner/utils.cpp, default_planner/TrajLNS.h,
default_planner/flow.cpp, default_planner/heuristics.cpp, default_planner/pibt.h, default_planner/scheduler.cpp,
default_planner/search.h, default_planner/utils.h, default_planner/Types.h, default_planner/flow.h,
default_planner/heuristics.h, default_planner/planner.cpp, default_planner/scheduler.h,  
python/common/MAPFbinding.cpp, python/default_planner/pyMAPFPlanner.cpp, 
python/default_scheduler/pyTaskScheduler.hpp, python/user_scheduler/pyTaskScheduler.cpp, python/common/pyEntry.hpp, python/default_planner/pyMAPFPlanner.hpp, python/user_planner/pyMAPFPlanner.cpp, 
python/user_scheduler/pyTaskScheduler.hpp, python/common/pyEnvironment.hpp, 
python/default_scheduler/pyTaskScheduler.cpp, python/user_planner/pyMAPFPlanner.hpp, python/set_track.bash        
```

In the **planner track**, don't modify or interfere with any start kit functionalities, including those in the following files:
```
inc/TaskScheduler.h, src/TaskScheduler.cpp, inc/Entry.h, src/Entry.cpp
```

In the **scheduler track**, don't modify or interfere with any start kit functionalities, including those in the following files:
```
inc/MAPFPlanner.h, src/MAPFPlanner.cpp, inc/Entry.h, src/Entry.cpp
```

## Evaluation Server Hardware Specification

Your submission will have access to a virtual machine with the following specifications:

**CPU Architecture**
```
Architecture:            x86_64
  CPU op-mode(s):        32-bit, 64-bit
  Address sizes:         48 bits physical, 48 bits virtual
  Byte Order:            Little Endian
CPU(s):                  32
  On-line CPU(s) list:   0-31
Vendor ID:               AuthenticAMD
  Model name:            AMD EPYC 7R32
    CPU family:          23
    Model:               49
    Thread(s) per core:  2
    Core(s) per socket:  16
    Socket(s):           1
    Stepping:            0
    BogoMIPS:            5600.00
    Flags:               fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge mca cmov pat pse36 clflush mmx fxsr sse sse2 ht syscall nx mmxext fxsr_opt pdpe1gb rdtscp lm constant_tsc rep_good no
                         pl nonstop_tsc cpuid extd_apicid aperfmperf tsc_known_freq pni pclmulqdq ssse3 fma cx16 sse4_1 sse4_2 movbe popcnt aes xsave avx f16c rdrand hypervisor lahf_lm cmp_legacy
                          cr8_legacy abm sse4a misalignsse 3dnowprefetch topoext ssbd ibrs ibpb stibp vmmcall fsgsbase bmi1 avx2 smep bmi2 rdseed adx smap clflushopt clwb sha_ni xsaveopt xsavec x
                         getbv1 clzero xsaveerptr rdpru wbnoinvd arat npt nrip_save rdpid

```

- Memory: 128 GB
- Storage: approximately 27GB free space.
- GPU NVIDIA A10G

## Docker Environment

The submission will be evaluated inside a Docker environment configured as follows:
- Base image: [pytorch/pytorch:2.4.1-cuda11.8-cudnn9-devel](https://hub.docker.com/layers/pytorch/pytorch/2.4.1-cuda11.8-cudnn9-devel/images/sha256-ebefd256e8247f1cea8f8cadd77f1944f6c3e65585c4e39a8d4135d29de4a0cb?context=explore)
- Operating System: Ubuntu 22.04.3 LTS (Jammy Jellyfish)
- Full access to all machine resources
- No internet access

### Python and Packages

If using the default `apt.txt` and `pip.txt` in the start-kit, the following packages will be available in the evaluation environment. 
If you added extra packages to `apt.txt` and `pip.txt`, then those packages will also be available.

By default, we have provided a Python environment with version 3.11.9 and Pybind11 with 2.13.6. All packages listed in the `pip.txt` will be installed to this default Python3.11. 
Note, however, that there is also a python3.10 provided by the Ubuntu operating system. Packages will not be installed for this environment and we recommend not to use it.

### Packages installed in default python
```
Package                   Version
------------------------- ------------
archspec                  0.2.3
asttokens                 2.4.1
astunparse                1.6.3
attrs                     24.2.0
beautifulsoup4            4.12.3
boltons                   24.0.0
Brotli                    1.1.0
certifi                   2024.8.30
cffi                      1.17.0
chardet                   5.2.0
charset-normalizer        3.3.2
click                     8.1.7
colorama                  0.4.6
conda                     24.7.1
conda-build               24.7.1
conda_index               0.5.0
conda-libmamba-solver     24.7.0
conda-package-handling    2.3.0
conda_package_streaming   0.10.0
decorator                 5.1.1
distro                    1.9.0
dnspython                 2.6.1
exceptiongroup            1.2.2
executing                 2.1.0
expecttest                0.2.1
filelock                  3.15.4
frozendict                2.4.4
fsspec                    2024.9.0
h2                        4.1.0
hpack                     4.0.0
hyperframe                6.0.1
hypothesis                6.111.2
idna                      3.8
importlib_resources       6.4.4
ipython                   8.27.0
jedi                      0.19.1
Jinja2                    3.1.4
jsonpatch                 1.33
jsonpointer               3.0.0
jsonschema                4.23.0
jsonschema-specifications 2023.12.1
libarchive-c              5.1
libmambapy                1.5.9
lief                      0.14.1
lintrunner                0.12.5
mamba                     1.5.9
MarkupSafe                2.1.5
matplotlib-inline         0.1.7
menuinst                  2.1.2
more-itertools            10.4.0
mpmath                    1.3.0
networkx                  3.3
ninja                     1.11.1.1
numpy                     2.1.1
nvidia-cublas-cu11        11.11.3.6
nvidia-cuda-cupti-cu11    11.8.87
nvidia-cuda-nvrtc-cu11    11.8.89
nvidia-cuda-runtime-cu11  11.8.89
nvidia-cudnn-cu11         9.1.0.70
nvidia-cufft-cu11         10.9.0.58
nvidia-curand-cu11        10.3.0.86
nvidia-cusolver-cu11      11.4.1.48
nvidia-cusparse-cu11      11.7.5.86
nvidia-nccl-cu11          2.20.5
nvidia-nvtx-cu11          11.8.86
optree                    0.12.1
packaging                 24.1
parso                     0.8.4
pexpect                   4.9.0
pickleshare               0.7.5
pillow                    10.2.0
pip                       24.2
pkginfo                   1.11.1
pkgutil_resolve_name      1.3.10
platformdirs              4.2.2
pluggy                    1.5.0
prompt_toolkit            3.0.47
psutil                    6.0.0
ptyprocess                0.7.0
pure_eval                 0.2.3
pybind11_global           2.13.6
pycosat                   0.6.6
pycparser                 2.22
Pygments                  2.18.0
PySocks                   1.7.1
python-etcd               0.4.5
pytz                      2024.1
PyYAML                    6.0.2
referencing               0.35.1
requests                  2.32.3
rpds-py                   0.20.0
ruamel.yaml               0.18.6
ruamel.yaml.clib          0.2.8
setuptools                75.1.0
six                       1.16.0
sortedcontainers          2.4.0
soupsieve                 2.5
stack-data                0.6.2
sympy                     1.13.2
torch                     2.4.1+cu118
torchaudio                2.4.1+cu118
torchelastic              0.2.2
torchvision               0.19.1+cu118
tqdm                      4.66.5
traitlets                 5.14.3
triton                    3.0.0
truststore                0.9.2
types-dataclasses         0.6.6
typing_extensions         4.12.2
urllib3                   2.2.2
wcwidth                   0.2.13
wheel                     0.44.0
zipp                      3.20.1
zstandard                 0.23.0
```


### Other packages managed by apt
```
adduser/jammy,now 3.118ubuntu5 all [installed]
apt/now 2.4.10 amd64 [installed,upgradable to: 2.4.13]
autoconf/jammy,now 2.71-2 all [installed,automatic]
automake/jammy,now 1:1.16.5-1.3 all [installed,automatic]
autotools-dev/jammy,now 20220109.1 all [installed,automatic]
base-files/now 12ubuntu4.4 amd64 [installed,upgradable to: 12ubuntu4.7]
base-passwd/jammy,now 3.5.52build1 amd64 [installed]
bash/jammy,now 5.1-6ubuntu1 amd64 [installed,upgradable to: 5.1-6ubuntu1.1]
binutils-common/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
binutils-x86-64-linux-gnu/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
binutils/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
bsdutils/jammy,now 1:2.37.2-4ubuntu3 amd64 [installed,upgradable to: 1:2.37.2-4ubuntu3.4]
build-essential/jammy,now 12.9ubuntu3 amd64 [installed]
bzip2/jammy,now 1.0.8-5build1 amd64 [installed,automatic]
ca-certificates/now 20230311ubuntu0.22.04.1 all [installed,upgradable to: 20240203~22.04.1]
cmake-data/jammy-updates,now 3.22.1-1ubuntu1.22.04.2 all [installed,automatic]
cmake/jammy-updates,now 3.22.1-1ubuntu1.22.04.2 amd64 [installed]
coreutils/jammy,now 8.32-4.1ubuntu1 amd64 [installed,upgradable to: 8.32-4.1ubuntu1.2]
cpp-11/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
cpp/jammy,now 4:11.2.0-1ubuntu1 amd64 [installed,automatic]
cuda-cccl-11-8/unknown,now 11.8.89-1 amd64 [installed,automatic]
cuda-command-line-tools-11-8/unknown,now 11.8.0-1 amd64 [installed]
cuda-compat-11-8/unknown,now 520.61.05-1 amd64 [installed]
cuda-compiler-11-8/unknown,now 11.8.0-1 amd64 [installed,automatic]
cuda-cudart-11-8/unknown,now 11.8.89-1 amd64 [installed]
cuda-cudart-dev-11-8/unknown,now 11.8.89-1 amd64 [installed]
cuda-cuobjdump-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-cupti-11-8/unknown,now 11.8.87-1 amd64 [installed,automatic]
cuda-cupti-dev-11-8/unknown,now 11.8.87-1 amd64 [installed,automatic]
cuda-cuxxfilt-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-driver-dev-11-8/unknown,now 11.8.89-1 amd64 [installed,automatic]
cuda-gdb-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-keyring/unknown,now 1.0-1 all [installed,upgradable to: 1.1-1]
cuda-libraries-11-8/unknown,now 11.8.0-1 amd64 [installed]
cuda-libraries-dev-11-8/unknown,now 11.8.0-1 amd64 [installed]
cuda-memcheck-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-minimal-build-11-8/unknown,now 11.8.0-1 amd64 [installed]
cuda-nsight-compute-11-8/unknown,now 11.8.0-1 amd64 [installed]
cuda-nvcc-11-8/unknown,now 11.8.89-1 amd64 [installed,automatic]
cuda-nvdisasm-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-nvml-dev-11-8/unknown,now 11.8.86-1 amd64 [installed]
cuda-nvprof-11-8/unknown,now 11.8.87-1 amd64 [installed]
cuda-nvprune-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-nvrtc-11-8/unknown,now 11.8.89-1 amd64 [installed,automatic]
cuda-nvrtc-dev-11-8/unknown,now 11.8.89-1 amd64 [installed,automatic]
cuda-nvtx-11-8/unknown,now 11.8.86-1 amd64 [installed]
cuda-profiler-api-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-sanitizer-11-8/unknown,now 11.8.86-1 amd64 [installed,automatic]
cuda-toolkit-11-8-config-common/unknown,now 11.8.89-1 all [installed,automatic]
cuda-toolkit-11-config-common/unknown,now 11.8.89-1 all [installed,automatic]
cuda-toolkit-config-common/unknown,now 12.3.52-1 all [installed,upgradable to: 12.6.77-1]
dash/jammy,now 0.5.11+git20210903+057cd650a4ed-3build1 amd64 [installed]
debconf/jammy,now 1.5.79ubuntu1 all [installed]
debianutils/jammy,now 5.5-1ubuntu2 amd64 [installed]
dh-elpa-helper/jammy,now 2.0.9ubuntu1 all [installed,automatic]
diffutils/jammy,now 1:3.8-0ubuntu2 amd64 [installed]
dirmngr/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
dpkg-dev/now 1.21.1ubuntu2.2 all [installed,upgradable to: 1.21.1ubuntu2.3]
dpkg/now 1.21.1ubuntu2.2 amd64 [installed,upgradable to: 1.21.1ubuntu2.3]
e2fsprogs/jammy-security,now 1.46.5-2ubuntu1.1 amd64 [installed,upgradable to: 1.46.5-2ubuntu1.2]
emacsen-common/jammy,now 3.0.4 all [installed,automatic]
findutils/jammy,now 4.8.0-1ubuntu3 amd64 [installed]
g++-11/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
g++/jammy,now 4:11.2.0-1ubuntu1 amd64 [installed,automatic]
gcc-11-base/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
gcc-11/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
gcc-12-base/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed]
gcc/jammy,now 4:11.2.0-1ubuntu1 amd64 [installed,automatic]
gfortran-11/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
gnupg-l10n/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 all [installed,automatic]
gnupg-utils/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gnupg2/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 all [installed]
gnupg/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 all [installed,automatic]
gpg-agent/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpg-wks-client/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpg-wks-server/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpg/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpgconf/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpgsm/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed,automatic]
gpgv/jammy-updates,jammy-security,now 2.2.27-3ubuntu2.1 amd64 [installed]
grep/jammy,now 3.7-1build1 amd64 [installed]
gzip/jammy-updates,now 1.10-4ubuntu4.1 amd64 [installed]
hostname/jammy,now 3.23ubuntu2 amd64 [installed]
ibverbs-providers/jammy,now 39.0-1 amd64 [installed,automatic]
icu-devtools/jammy,now 70.1-2 amd64 [installed,automatic]
init-system-helpers/jammy,now 1.62 all [installed]
libacl1/jammy,now 2.3.1-1 amd64 [installed]
libapt-pkg6.0/now 2.4.10 amd64 [installed,upgradable to: 2.4.13]
libarchive13/jammy-updates,jammy-security,now 3.6.0-1ubuntu1.1 amd64 [installed,automatic]
libasan6/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
libassuan0/jammy,now 2.5.5-1build1 amd64 [installed,automatic]
libatomic1/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libattr1/jammy,now 1:2.5.1-1build1 amd64 [installed]
libaudit-common/jammy,now 1:3.0.7-1build1 all [installed]
libaudit1/jammy,now 1:3.0.7-1build1 amd64 [installed]
libbinutils/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
libblkid1/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
libboost-all-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed]
libboost-atomic-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-atomic1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-atomic1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-chrono-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-chrono1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-chrono1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-container-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-container1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-container1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-context-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-context1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-context1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-coroutine-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-coroutine1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-coroutine1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-date-time-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-date-time1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-date-time1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-exception-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-exception1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-fiber-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-fiber1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-fiber1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-filesystem-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-filesystem1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-filesystem1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-graph-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-graph-parallel-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-graph-parallel1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-graph-parallel1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-graph1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-graph1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-iostreams-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-iostreams1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-iostreams1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-locale-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-locale1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-locale1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-log-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-log1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-log1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-math-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-math1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-math1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-mpi-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-mpi-python-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-mpi-python1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-mpi-python1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-mpi1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-mpi1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-nowide-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-nowide1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-nowide1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-numpy-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-numpy1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-numpy1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-program-options-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-program-options1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-program-options1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-python-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-python1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-python1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-random-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-random1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-random1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-regex-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-regex1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-regex1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-serialization-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-serialization1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-serialization1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-stacktrace-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-stacktrace1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-stacktrace1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-system-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-system1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-system1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-test-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-test1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-test1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-thread-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-thread1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-thread1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-timer-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-timer1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-timer1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-tools-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-type-erasure-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-type-erasure1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-type-erasure1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-wave-dev/jammy,now 1.74.0.3ubuntu7 amd64 [installed,automatic]
libboost-wave1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost-wave1.74.0/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost1.74-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libboost1.74-tools-dev/jammy,now 1.74.0-14ubuntu3 amd64 [installed,automatic]
libbrotli1/jammy,now 1.0.9-2build6 amd64 [installed,automatic]
libbsd0/jammy,now 0.11.5-1 amd64 [installed,automatic]
libbz2-1.0/jammy,now 1.0.8-5build1 amd64 [installed]
libc-bin/now 2.35-0ubuntu3.4 amd64 [installed,upgradable to: 2.35-0ubuntu3.8]
libc-dev-bin/now 2.35-0ubuntu3.4 amd64 [installed,upgradable to: 2.35-0ubuntu3.8]
libc6-dev/now 2.35-0ubuntu3.4 amd64 [installed,upgradable to: 2.35-0ubuntu3.8]
libc6/now 2.35-0ubuntu3.4 amd64 [installed,upgradable to: 2.35-0ubuntu3.8]
libcap-ng0/jammy,now 0.7.9-2.2build3 amd64 [installed]
libcap2/jammy-updates,jammy-security,now 1:2.44-1ubuntu0.22.04.1 amd64 [installed]
libcbor0.8/jammy,now 0.8.0-2ubuntu1 amd64 [installed,automatic]
libcc1-0/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libcom-err2/jammy-security,now 1.46.5-2ubuntu1.1 amd64 [installed,upgradable to: 1.46.5-2ubuntu1.2]
libcrypt-dev/jammy,now 1:4.4.27-1 amd64 [installed,automatic]
libcrypt1/jammy,now 1:4.4.27-1 amd64 [installed]
libctf-nobfd0/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
libctf0/now 2.38-4ubuntu2.3 amd64 [installed,upgradable to: 2.38-4ubuntu2.6]
libcublas-11-8/unknown,now 11.11.3.6-1 amd64 [installed]
libcublas-dev-11-8/unknown,now 11.11.3.6-1 amd64 [installed]
libcufft-11-8/unknown,now 10.9.0.58-1 amd64 [installed,automatic]
libcufft-dev-11-8/unknown,now 10.9.0.58-1 amd64 [installed,automatic]
libcufile-11-8/unknown,now 1.4.0.31-1 amd64 [installed,automatic]
libcufile-dev-11-8/unknown,now 1.4.0.31-1 amd64 [installed,automatic]
libcurand-11-8/unknown,now 10.3.0.86-1 amd64 [installed,automatic]
libcurand-dev-11-8/unknown,now 10.3.0.86-1 amd64 [installed,automatic]
libcurl3-gnutls/jammy-updates,jammy-security,now 7.81.0-1ubuntu1.18 amd64 [installed,automatic]
libcurl4/jammy-updates,jammy-security,now 7.81.0-1ubuntu1.18 amd64 [installed,automatic]
libcusolver-11-8/unknown,now 11.4.1.48-1 amd64 [installed,automatic]
libcusolver-dev-11-8/unknown,now 11.4.1.48-1 amd64 [installed,automatic]
libcusparse-11-8/unknown,now 11.7.5.86-1 amd64 [installed]
libcusparse-dev-11-8/unknown,now 11.7.5.86-1 amd64 [installed]
libdb5.3/jammy,now 5.3.28+dfsg1-0.8ubuntu3 amd64 [installed]
libdebconfclient0/jammy,now 0.261ubuntu1 amd64 [installed]
libdpkg-perl/now 1.21.1ubuntu2.2 all [installed,upgradable to: 1.21.1ubuntu2.3]
libedit2/jammy,now 3.1-20210910-1build1 amd64 [installed,automatic]
libevent-2.1-7/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libevent-core-2.1-7/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libevent-dev/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libevent-extra-2.1-7/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libevent-openssl-2.1-7/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libevent-pthreads-2.1-7/jammy,now 2.1.12-stable-1build3 amd64 [installed,automatic]
libexpat1-dev/jammy-updates,jammy-security,now 2.4.7-1ubuntu0.4 amd64 [installed,automatic]
libexpat1/jammy-updates,jammy-security,now 2.4.7-1ubuntu0.4 amd64 [installed,automatic]
libext2fs2/jammy-security,now 1.46.5-2ubuntu1.1 amd64 [installed,upgradable to: 1.46.5-2ubuntu1.2]
libfabric1/jammy,now 1.11.0-3 amd64 [installed,automatic]
libffi8/jammy,now 3.4.2-4 amd64 [installed]
libfido2-1/jammy,now 1.10.0-1 amd64 [installed,automatic]
libgcc-11-dev/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
libgcc-s1/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed]
libgcrypt20/jammy,now 1.9.4-3ubuntu3 amd64 [installed]
libgdbm-compat4/jammy,now 1.23-1 amd64 [installed,automatic]
libgdbm6/jammy,now 1.23-1 amd64 [installed,automatic]
libgfortran-11-dev/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
libgfortran5/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libgmp10/jammy,now 2:6.2.1+dfsg-3ubuntu1 amd64 [installed]
libgnutls30/now 3.7.3-4ubuntu1.2 amd64 [installed,upgradable to: 3.7.3-4ubuntu1.5]
libgomp1/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libgpg-error0/jammy,now 1.43-3 amd64 [installed]
libgssapi-krb5-2/now 1.19.2-2ubuntu0.2 amd64 [installed,upgradable to: 1.19.2-2ubuntu0.4]
libhogweed6/jammy,now 3.7.3-1build2 amd64 [installed]
libhwloc-dev/jammy-updates,now 2.7.0-2ubuntu1 amd64 [installed,automatic]
libhwloc-plugins/jammy-updates,now 2.7.0-2ubuntu1 amd64 [installed,automatic]
libhwloc15/jammy-updates,now 2.7.0-2ubuntu1 amd64 [installed,automatic]
libibverbs-dev/jammy,now 39.0-1 amd64 [installed,automatic]
libibverbs1/jammy,now 39.0-1 amd64 [installed,automatic]
libicu-dev/jammy,now 70.1-2 amd64 [installed,automatic]
libicu70/jammy,now 70.1-2 amd64 [installed,automatic]
libidn2-0/jammy,now 2.3.2-2build1 amd64 [installed]
libisl23/jammy,now 0.24-2build1 amd64 [installed,automatic]
libitm1/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libjpeg-dev/jammy,now 8c-2ubuntu10 amd64 [installed]
libjpeg-turbo8-dev/jammy,now 2.1.2-0ubuntu1 amd64 [installed,automatic]
libjpeg-turbo8/jammy,now 2.1.2-0ubuntu1 amd64 [installed,automatic]
libjpeg8-dev/jammy,now 8c-2ubuntu10 amd64 [installed,automatic]
libjpeg8/jammy,now 8c-2ubuntu10 amd64 [installed,automatic]
libjs-jquery-ui/jammy,now 1.13.1+dfsg-1 all [installed,automatic]
libjs-jquery/jammy,now 3.6.0+dfsg+~3.5.13-1 all [installed,automatic]
libjs-sphinxdoc/jammy,now 4.3.2-1 all [installed,automatic]
libjs-underscore/jammy,now 1.13.2~dfsg-2 all [installed,automatic]
libjsoncpp25/jammy,now 1.9.5-3 amd64 [installed,automatic]
libk5crypto3/now 1.19.2-2ubuntu0.2 amd64 [installed,upgradable to: 1.19.2-2ubuntu0.4]
libkeyutils1/jammy,now 1.6.1-2ubuntu3 amd64 [installed]
libkrb5-3/now 1.19.2-2ubuntu0.2 amd64 [installed,upgradable to: 1.19.2-2ubuntu0.4]
libkrb5support0/now 1.19.2-2ubuntu0.2 amd64 [installed,upgradable to: 1.19.2-2ubuntu0.4]
libksba8/jammy-updates,jammy-security,now 1.6.0-2ubuntu0.2 amd64 [installed,automatic]
libldap-2.5-0/now 2.5.16+dfsg-0ubuntu0.22.04.1 amd64 [installed,upgradable to: 2.5.18+dfsg-0ubuntu0.22.04.2]
liblsan0/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libltdl-dev/jammy,now 2.4.6-15build2 amd64 [installed,automatic]
libltdl7/jammy,now 2.4.6-15build2 amd64 [installed,automatic]
liblz4-1/jammy,now 1.9.3-2build2 amd64 [installed]
liblzma5/jammy,now 5.2.5-2ubuntu1 amd64 [installed]
libmd0/jammy,now 1.0.4-1build1 amd64 [installed,automatic]
libmount1/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
libmpc3/jammy,now 1.2.1-2build1 amd64 [installed,automatic]
libmpdec3/jammy,now 2.5.1-2build2 amd64 [installed,automatic]
libmpfr6/jammy,now 4.1.0-3build3 amd64 [installed,automatic]
libnccl-dev/unknown,unknown,now 2.15.5-1+cuda11.8 amd64 [installed,upgradable to: 2.23.4-1+cuda12.6]
libnccl2/unknown,unknown,now 2.15.5-1+cuda11.8 amd64 [installed,upgradable to: 2.23.4-1+cuda12.6]
libncurses6/jammy-updates,jammy-security,now 6.3-2ubuntu0.1 amd64 [installed]
libncursesw6/jammy-updates,jammy-security,now 6.3-2ubuntu0.1 amd64 [installed]
libnettle8/jammy,now 3.7.3-1build2 amd64 [installed]
libnghttp2-14/jammy-updates,jammy-security,now 1.43.0-1ubuntu0.2 amd64 [installed,automatic]
libnl-3-200/jammy,now 3.5.0-0.1 amd64 [installed,automatic]
libnl-3-dev/jammy,now 3.5.0-0.1 amd64 [installed,automatic]
libnl-route-3-200/jammy,now 3.5.0-0.1 amd64 [installed,automatic]
libnl-route-3-dev/jammy,now 3.5.0-0.1 amd64 [installed,automatic]
libnpp-11-8/unknown,now 11.8.0.86-1 amd64 [installed]
libnpp-dev-11-8/unknown,now 11.8.0.86-1 amd64 [installed]
libnpth0/jammy,now 1.6-3build2 amd64 [installed,automatic]
libnsl-dev/jammy,now 1.3.0-2build2 amd64 [installed,automatic]
libnsl2/jammy,now 1.3.0-2build2 amd64 [installed]
libnuma-dev/jammy,now 2.0.14-3ubuntu2 amd64 [installed,automatic]
libnuma1/jammy,now 2.0.14-3ubuntu2 amd64 [installed,automatic]
libnvjpeg-11-8/unknown,now 11.9.0.86-1 amd64 [installed,automatic]
libnvjpeg-dev-11-8/unknown,now 11.9.0.86-1 amd64 [installed,automatic]
libopenmpi-dev/jammy,now 4.1.2-2ubuntu1 amd64 [installed,automatic]
libopenmpi3/jammy,now 4.1.2-2ubuntu1 amd64 [installed,automatic]
libp11-kit0/jammy,now 0.24.0-6build1 amd64 [installed]
libpam-modules-bin/now 1.4.0-11ubuntu2.3 amd64 [installed,upgradable to: 1.4.0-11ubuntu2.4]
libpam-modules/now 1.4.0-11ubuntu2.3 amd64 [installed,upgradable to: 1.4.0-11ubuntu2.4]
libpam-runtime/now 1.4.0-11ubuntu2.3 all [installed,upgradable to: 1.4.0-11ubuntu2.4]
libpam0g/now 1.4.0-11ubuntu2.3 amd64 [installed,upgradable to: 1.4.0-11ubuntu2.4]
libpciaccess0/jammy,now 0.16-3 amd64 [installed,automatic]
libpcre2-8-0/jammy-updates,jammy-security,now 10.39-3ubuntu0.1 amd64 [installed]
libpcre3/jammy-updates,jammy-security,now 2:8.39-13ubuntu0.22.04.1 amd64 [installed]
libperl5.34/now 5.34.0-3ubuntu1.2 amd64 [installed,upgradable to: 5.34.0-3ubuntu1.3]
libpmix-dev/jammy,now 4.1.2-2ubuntu1 amd64 [installed,automatic]
libpmix2/jammy,now 4.1.2-2ubuntu1 amd64 [installed,automatic]
libpng-dev/jammy,now 1.6.37-3build5 amd64 [installed]
libpng16-16/jammy,now 1.6.37-3build5 amd64 [installed,automatic]
libprocps8/jammy,now 2:3.3.17-6ubuntu2 amd64 [installed,upgradable to: 2:3.3.17-6ubuntu2.1]
libpsl5/jammy,now 0.21.0-1.2build2 amd64 [installed,automatic]
libpsm-infinipath1/jammy,now 3.3+20.604758e7-6.1 amd64 [installed,automatic]
libpsm2-2/jammy,now 11.2.185-1 amd64 [installed,automatic]
libpython3-dev/jammy-updates,now 3.10.6-1~22.04.1 amd64 [installed,automatic]
libpython3-stdlib/jammy-updates,now 3.10.6-1~22.04.1 amd64 [installed,automatic]
libpython3.10-dev/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
libpython3.10-minimal/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
libpython3.10-stdlib/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
libpython3.10/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
libquadmath0/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
librdmacm1/jammy,now 39.0-1 amd64 [installed,automatic]
libreadline8/jammy,now 8.1.2-1 amd64 [installed,automatic]
librhash0/jammy,now 1.4.2-1ubuntu1 amd64 [installed,automatic]
librtmp1/jammy,now 2.4+20151223.gitfa8646d.1-2build4 amd64 [installed,automatic]
libsasl2-2/jammy-updates,now 2.1.27+dfsg2-3ubuntu1.2 amd64 [installed,automatic]
libsasl2-modules-db/jammy-updates,now 2.1.27+dfsg2-3ubuntu1.2 amd64 [installed,automatic]
libseccomp2/jammy,now 2.5.3-2ubuntu2 amd64 [installed]
libselinux1/jammy,now 3.3-1build2 amd64 [installed]
libsemanage-common/jammy,now 3.3-1build2 all [installed]
libsemanage2/jammy,now 3.3-1build2 amd64 [installed]
libsepol2/jammy,now 3.3-1build1 amd64 [installed]
libsigsegv2/jammy,now 2.13-1ubuntu3 amd64 [installed,automatic]
libsmartcols1/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
libsqlite3-0/now 3.37.2-2ubuntu0.1 amd64 [installed,upgradable to: 3.37.2-2ubuntu0.3]
libss2/jammy-security,now 1.46.5-2ubuntu1.1 amd64 [installed,upgradable to: 1.46.5-2ubuntu1.2]
libssh-4/jammy-updates,jammy-security,now 0.9.6-2ubuntu0.22.04.3 amd64 [installed,automatic]
libssl3/now 3.0.2-0ubuntu1.10 amd64 [installed,upgradable to: 3.0.2-0ubuntu1.18]
libstdc++-11-dev/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
libstdc++6/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed]
libsystemd0/now 249.11-0ubuntu3.10 amd64 [installed,upgradable to: 249.11-0ubuntu3.12]
libtasn1-6/jammy,now 4.18.0-4build1 amd64 [installed]
libtinfo6/jammy-updates,jammy-security,now 6.3-2ubuntu0.1 amd64 [installed]
libtirpc-common/jammy-updates,jammy-security,now 1.3.2-2ubuntu0.1 all [installed]
libtirpc-dev/jammy-updates,jammy-security,now 1.3.2-2ubuntu0.1 amd64 [installed,automatic]
libtirpc3/jammy-updates,jammy-security,now 1.3.2-2ubuntu0.1 amd64 [installed]
libtsan0/jammy-updates,jammy-security,now 11.4.0-1ubuntu1~22.04 amd64 [installed,automatic]
libubsan1/jammy-updates,jammy-security,now 12.3.0-1ubuntu1~22.04 amd64 [installed,automatic]
libucx0/jammy,now 1.12.1~rc2-1 amd64 [installed,automatic]
libudev1/now 249.11-0ubuntu3.10 amd64 [installed,upgradable to: 249.11-0ubuntu3.12]
libunistring2/jammy,now 1.0-1 amd64 [installed]
libuuid1/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
libuv1/jammy-updates,jammy-security,now 1.43.0-1ubuntu0.1 amd64 [installed,automatic]
libx11-6/jammy-updates,jammy-security,now 2:1.7.5-1ubuntu0.3 amd64 [installed,automatic]
libx11-data/jammy-updates,jammy-security,now 2:1.7.5-1ubuntu0.3 all [installed,automatic]
libxau6/jammy,now 1:1.0.9-1build5 amd64 [installed,automatic]
libxcb1/jammy,now 1.14-3ubuntu3 amd64 [installed,automatic]
libxdmcp6/jammy,now 1:1.1.3-0ubuntu5 amd64 [installed,automatic]
libxext6/jammy,now 2:1.3.4-1build1 amd64 [installed,automatic]
libxml2/jammy-updates,jammy-security,now 2.9.13+dfsg-1ubuntu0.4 amd64 [installed,automatic]
libxnvctrl0/unknown,now 560.35.03-0ubuntu1 amd64 [installed,automatic]
libxxhash0/jammy,now 0.8.1-1 amd64 [installed]
libzstd1/jammy,now 1.4.8+dfsg-3build1 amd64 [installed]
linux-libc-dev/now 5.15.0-88.98 amd64 [installed,upgradable to: 5.15.0-122.132]
login/now 1:4.8.1-2ubuntu2.1 amd64 [installed,upgradable to: 1:4.8.1-2ubuntu2.2]
logsave/jammy-security,now 1.46.5-2ubuntu1.1 amd64 [installed,upgradable to: 1.46.5-2ubuntu1.2]
lsb-base/jammy,now 11.1.0ubuntu4 all [installed]
lto-disabled-list/jammy,now 24 all [installed,automatic]
m4/jammy,now 1.4.18-5ubuntu2 amd64 [installed,automatic]
make/jammy,now 4.3-4.1build1 amd64 [installed,automatic]
mawk/jammy,now 1.3.4.20200120-3 amd64 [installed]
media-types/jammy,now 7.0.0 all [installed,automatic]
mount/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
mpi-default-bin/jammy,now 1.14 amd64 [installed,automatic]
mpi-default-dev/jammy,now 1.14 amd64 [installed,automatic]
ncurses-base/jammy-updates,jammy-security,now 6.3-2ubuntu0.1 all [installed]
ncurses-bin/jammy-updates,jammy-security,now 6.3-2ubuntu0.1 amd64 [installed]
nsight-compute-2022.3.0/unknown,now 2022.3.0.22-1 amd64 [installed,automatic]
ocl-icd-libopencl1/jammy,now 2.2.14-3 amd64 [installed,automatic]
openmpi-bin/jammy,now 4.1.2-2ubuntu1 amd64 [installed,automatic]
openmpi-common/jammy,now 4.1.2-2ubuntu1 all [installed,automatic]
openssh-client/jammy-updates,jammy-security,now 1:8.9p1-3ubuntu0.10 amd64 [installed,automatic]
openssl/now 3.0.2-0ubuntu1.12 amd64 [installed,upgradable to: 3.0.2-0ubuntu1.18]
passwd/now 1:4.8.1-2ubuntu2.1 amd64 [installed,upgradable to: 1:4.8.1-2ubuntu2.2]
patch/jammy,now 2.7.6-7build2 amd64 [installed,automatic]
perl-base/now 5.34.0-3ubuntu1.2 amd64 [installed,upgradable to: 5.34.0-3ubuntu1.3]
perl-modules-5.34/now 5.34.0-3ubuntu1.2 all [installed,upgradable to: 5.34.0-3ubuntu1.3]
perl/now 5.34.0-3ubuntu1.2 amd64 [installed,upgradable to: 5.34.0-3ubuntu1.3]
pinentry-curses/jammy,now 1.1.1-1build2 amd64 [installed,automatic]
procps/jammy,now 2:3.3.17-6ubuntu2 amd64 [installed,upgradable to: 2:3.3.17-6ubuntu2.1]
python3-dev/jammy-updates,now 3.10.6-1~22.04.1 amd64 [installed,automatic]
python3-distutils/jammy-updates,jammy-security,now 3.10.8-1~22.04 all [installed,automatic]
python3-lib2to3/jammy-updates,jammy-security,now 3.10.8-1~22.04 all [installed,automatic]
python3-minimal/jammy-updates,now 3.10.6-1~22.04.1 amd64 [installed,automatic]
python3.10-dev/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
python3.10-minimal/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
python3.10/jammy-updates,jammy-security,now 3.10.12-1~22.04.6 amd64 [installed,automatic]
python3/jammy-updates,now 3.10.6-1~22.04.1 amd64 [installed,automatic]
readline-common/jammy,now 8.1.2-1 all [installed,automatic]
rpcsvc-proto/jammy,now 1.4.2-0ubuntu6 amd64 [installed,automatic]
sed/jammy,now 4.8-1ubuntu2 amd64 [installed]
sensible-utils/jammy,now 0.0.17 all [installed]
sysvinit-utils/jammy,now 3.01-1ubuntu1 amd64 [installed]
tar/now 1.34+dfsg-1ubuntu0.1.22.04.1 amd64 [installed,upgradable to: 1.34+dfsg-1ubuntu0.1.22.04.2]
ubuntu-keyring/jammy,now 2021.03.26 all [installed]
usrmerge/jammy,now 25ubuntu2 all [installed]
util-linux/jammy,now 2.37.2-4ubuntu3 amd64 [installed,upgradable to: 2.37.2-4ubuntu3.4]
xz-utils/jammy,now 5.2.5-2ubuntu1 amd64 [installed,automatic]
zlib1g-dev/jammy-updates,jammy-security,now 1:1.2.11.dfsg-2ubuntu9.2 amd64 [installed,automatic]
zlib1g/jammy-updates,jammy-security,now 1:1.2.11.dfsg-2ubuntu9.2 amd64 [installed]
```



