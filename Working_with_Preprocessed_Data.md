# Working with Preprocessed Data

Some algorithms have an offline preprocessing step to compute auxiliary data. The pre-computed data is then loaded and made available to improve the performance of online planning. 
Examples of auxiliary data include machine learning models (weights), strong bounding heuristics and sophisticated constraints that prune the search space.
These can be computed during the preprocessing step or loaded from your `large file storage` location.
Another situation is large size dependencies which are required during compilation.
In this document, we explain how to work with such auxiliary data and dependencies if your submission makes use of them.

## Generate your auxiliary data

The creation of auxiliary data is the responsibility of each participant. We assume you have external programs that can read and analyse problem instances. These programs run locally (i.e., on your machines or cloud instances) and they can take as long as required (e.g., days). 
They may also interact with your planner, but you should not implement these procedures directly in your planner. 
Instead, your planner should load the resulting auxiliary data during the evaluation of each submission, as part of our 30-minute preprocessing step. 

After the competition, we will open-source your submission. For this reason, we recommend you add the code to generate the auxiliary data to your submission repository. This will allow members of the community to replicate your results in the future and may lead to wider adoption of your implementation.

## Load your auxiliary data

Suppose, your auxiliary data is called `example.data` and is located on your local machine in directory `/tmp`. 
You will need to make this file available to your planner:
- The start-kit argument `--fileStoragePath` can be used to specify the location of the directory where the auxiliary data lives, in this example `--fileStoragePath /tmp`:
- If `--fileStoragePath` is not specified, the start-kit will first read the value of the environment variable `LORR_LARGE_FILE_STORAGE_PATH`.
The path value will be stored at `env->file_storage_path`, which your program can access in initalisation and planning.

Consider the following example:
```
./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json --fileStoragePath /tmp
```

Before evaluation, the start-kit environment calls `MAPFPlanner::initialize` function, which you will implement. 
Locate your auxiliary data by reading the variable `MAPFPlanner::env->file_storage_path`. For example, to load `example.data`, your planner can read 
`
path  = env->file_storage_path + "/example.data"
`.
You now have 30 minutes to load your auxiliary data file and initialise supporting data structures.
These will then be used by your planner during online evaluation. 

## Store your auxiliary data

Auxiliary data files can be quite large and should not be added to your submission repository.
Instead, we provide a large file hosting service to achieve the same result.

Navigate to the `My Submission` page:
- A `Large File Storage` button can be found at the bottom of the page.
- Click the button to open a storage management panel.
- Upload or delete files using this panel.

The uploaded file will be synced to the evaluation server when the evaluation starts.
The folder that stores these files will be mounted to the docker container with read-only access.
All of your submissions have access to this folder.
If a particular submission requires updated auxiliary data, you will need to manually delete and re-upload via the `My Submission` page.

## Dependency management using the large file storage 

Sometimes program have large compilation-time dependencies that cannot be uploaded to the submission repository. You can also use the large file storage to store to help in these situations. At compile time, we will initialise the environment variable `LORR_LARGE_FILE_STORAGE_PATH` with the location of your large file storage folder. You can refer to this variable in your compilation script to look for your dependencies.
