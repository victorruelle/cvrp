from distutils.core import setup, Extension

module1 = Extension('cvrpsep',
                    sources = ['BASEGRPH.cpp', 'CAPSEP.cpp', 'CNSTRMGR.cpp', 'COMPCUTS.cpp', 'COMPRESS.cpp', 'CUTBASE.cpp',
                               'FCAPFIX.cpp', 'FCISEP.cpp', 'GRSEARCH.cpp', 'HPMSTAR.cpp', 'MEMMOD.cpp',
                               'MSTARSEP.cpp', 'MXF.cpp', 'NEWHTOUR.cpp', 'SORT.cpp', 'STRCOMB.cpp', 'STRNGCMP.cpp', 'TWOMATCH.cpp',
                               'intap.cpp', 'blocks.cpp', 'BinPack.cpp', 'brnching.cpp', 'combsep.cpp', 'fcits.cpp', 'glmsep.cpp',
                               'htoursep.cpp', 'Python.cpp']
                    )

setup(name = 'cvrpsep', version = '1.0', description = 'CVRPSEP programs', ext_modules = [module1])
