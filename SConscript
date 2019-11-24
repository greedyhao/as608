from building import * 

# get current dir path
cwd = GetCurrentDir()

src = []
inc = [cwd]

src += ['as608.c']

if GetDepend(['PKG_USING_AS608_SAMPLE']):
    src += ['as608_sample.c']

group = DefineGroup('as608', src, depend = ['PKG_USING_AS608'], CPPPATH = inc)
Return('group')
