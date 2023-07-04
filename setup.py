from setuptools import setup, Extension
import pybind11
from pybind11.setup_helpers import Pybind11Extension


cpp_args=["-std=c++17"]

functions_module = Extension(
    name='MAPF',
    include_dirs=[pybind11.get_include(),"./inc/"],
    sources=['src/Grid.cpp',
        'src/States.cpp',
        "python/MAPFbinding.cpp"
        ],
    language='c++',
    extra_compile_args=cpp_args,
)

setup(name="MAPF",ext_modules=[functions_module])
