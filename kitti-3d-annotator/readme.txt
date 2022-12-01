work in windows, 
buggy in linux:
	viewer() hang
	fix: rm libz.so.1 come with pptk, in ldls env, rm /media/student/data5/.pyenv/versions/miniconda3-latest/envs/ldls/lib/python3.7/site-packages/pptk/libs/libz.so.1
	this will make pptk to use system libz.so.1
