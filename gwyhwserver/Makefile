SOURCE_FILES=cal_adc18b.c\
	     cal_inputs.c\
	     cal_outputs.c\
	     clientserver.h\
	     cmirp.c\
	     cmirp.h\
             communication.c\
             communication.h\
	     confuseacat.c\
	     data.c\
	     data.h\
	     gwyfile.c\
	     gwyfile.h\
	     hwserver.c\
	     ini.c\
	     ini.h\
	     luafunctions.c\
	     luafunctions.h\
	     stabilizer.c \
	     stabilizer.h \
	     fake.h \
	     fake.c \
	     myrpfake.c\
	     myrpfake.h\
             rptable.c\
             rptable.h\
             simplelink.c\
             simplelink.h\
	     statistics.c\
	     statistics.h\
	     test_akiyama.c\
	     test_client.c\
	     test_dac.c\
	     test_feedback.c\
             test_fpgaspi.c\
             test_fpgaspiint.c\
             test_kelvin.c\
	     test_lockin.c\
	     test_log_akiyama.c\
	     test_nenoprobe.c\
	     test_output.c\
	     test_spi.c\
	     test_tapping.c

DATA_FILES=fakea.txt\
	   fakeb.txt\
	   fakez.txt

GNUPLOT_FILES=fit_data.gpl\
	      fit_data_bare.gpl\
	      fit_data_out.gpl\
	      fit_data_bare_out.gpl

all: hwserver test_spi test_fpgaspi test_fpgaspiint cal_inputs cal_outputs test_akiyama test_kelvin cal_adc18b test_feedback
#all: hwserver test_spi test_feedback test_client test_lockin test_akiyama test_nenoprobe test_output test_dac test_tapping test_log_akiyama confuseacat test_kelvin

#FAKEFLAGS=-DFAKE=1
FAKEFLAGS=-UFAKE
	
hwserverfake: hwserver.o gwyfile.o data.o myrpfake.o luafunctions.o ini.o
	gcc -o hwserver -pthread -g hwserver.o gwyfile.o data.o myrpfake.o luafunctions.o ini.o -L../api/lib -L/home/valtr/run/lua-5.3.1/lib/ -llua5.3 -lrt -lm

hwserver: hwserver.o gwyfile.o data.o cmirp.o luafunctions.o ini.o rptable.o communication.o statistics.o simplelink.o stabilizer.o fake.o
	gcc -o hwserver -pthread -g hwserver.o gwyfile.o data.o cmirp.o luafunctions.o ini.o rptable.o statistics.o communication.o simplelink.o stabilizer.o fake.o -L../api/lib -L/home/valtr/run/lua-5.3.1/lib/ -llua5.3 -lrt -lm -lfftw3

test_client: test_client.o gwyfile.o
	gcc -o test_client test_client.o gwyfile.o -lm  -g

test_client.o: test_client.c gwyfile.h
	gcc -c test_client.c -Wall -g

test_spi: test_spi.o cmirp.o
	gcc -o test_spi test_spi.o cmirp.o -lm  -g

test_spi.o: test_spi.c cmirp.h
	gcc -c test_spi.c -Wall -g

test_fpgaspi: test_fpgaspi.o cmirp.o
	gcc -o test_fpgaspi test_fpgaspi.o cmirp.o -lrt -lm  -g

test_fpgaspi.o: test_fpgaspi.c cmirp.h
	gcc -c test_fpgaspi.c -Wall -g

test_fpgaspiint: test_fpgaspiint.o cmirp.o
	gcc -o test_fpgaspiint test_fpgaspiint.o cmirp.o -lrt -lm  -g

test_fpgaspiint.o: test_fpgaspiint.c cmirp.h
	gcc -c test_fpgaspiint.c -Wall -g

test_feedback: test_feedback.o cmirp.o
	gcc -o test_feedback test_feedback.o cmirp.o -lrt -lm  -g

test_feedback.o: test_feedback.c cmirp.h
	gcc -c test_feedback.c -Wall -g

test_kelvin: test_kelvin.o cmirp.o
	gcc -o test_kelvin test_kelvin.o cmirp.o -lrt -lm  -g

test_kelvin.o: test_kelvin.c cmirp.h
	gcc -c test_kelvin.c -Wall -g

test_nenoprobe: test_nenoprobe.o cmirp.o
	gcc -o test_nenoprobe test_nenoprobe.o cmirp.o -lrt -lm  -g

test_nenoprobe.o: test_nenoprobe.c cmirp.h
	gcc -c test_nenoprobe.c -Wall -g

test_lockin: test_lockin.o cmirp.o
	gcc -o test_lockin test_lockin.o cmirp.o -lrt -lm  -g

test_lockin.o: test_lockin.c cmirp.h
	gcc -c test_lockin.c -Wall -g

test_tapping: test_tapping.o cmirp.o
	gcc -o test_tapping test_tapping.o cmirp.o -lrt -lm  -g

test_tapping.o: test_tapping.c cmirp.h
	gcc -c test_tapping.c -Wall -g

test_akiyama: test_akiyama.o cmirp.o
	gcc -o test_akiyama test_akiyama.o cmirp.o -lrt -lm  -g

test_akiyama.o: test_akiyama.c cmirp.h
	gcc -c test_akiyama.c -Wall -g

confuseacat: confuseacat.o cmirp.o
	gcc -o confuseacat confuseacat.o cmirp.o -lrt -lm  -g

confuseacat.o: confuseacat.c cmirp.h
	gcc -c confuseacat.c -Wall -g

test_log_akiyama: test_log_akiyama.o cmirp.o
	gcc -o test_log_akiyama test_log_akiyama.o cmirp.o -lrt -lm  -g

test_log_akiyama.o: test_log_akiyama.c cmirp.h
	gcc -c test_log_akiyama.c -Wall -g

test_output: test_output.o cmirp.o
	gcc -o test_output test_output.o cmirp.o -lrt -lm  -g

test_output.o: test_output.c cmirp.h
	gcc -c test_output.c -Wall -g

test_dac: test_dac.o cmirp.o
	gcc -o test_dac test_dac.o cmirp.o -lrt -lm  -g

test_dac.o: test_dac.c cmirp.h
	gcc -c test_dac.c -Wall -g

hwserver.o: hwserver.c gwyfile.h clientserver.h communication.h cmirp.h data.h luafunctions.h ini.h myrpfake.h stabilizer.c fake.c
	gcc -I../api/include -c hwserver.c -pthread -Wall -g -I/usr/include/lua5.3 -I/home/valtr/run/lua-5.3.1/include $(FAKEFLAGS)

data.o: data.c data.h gwyfile.h clientserver.h cmirp.h myrpfake.h 
	gcc -c data.c -I/usr/include/lua5.3/ -Wall -g $(FAKEFLAGS)

rptable.o: rptable.c rptable.h clientserver.h
	gcc -c rptable.c -Wall -g

luafunctions.o: luafunctions.c luafunctions.h data.h
	gcc -c luafunctions.c -I/usr/include/lua5.3/ -Wall -g

ini.o: ini.c ini.h
	gcc -c ini.c -Wall -g

gwyfile.o: gwyfile.c gwyfile.h
	gcc -c gwyfile.c -Wall -g

communication.o: communication.c gwyfile.h clientserver.h cmirp.h data.h rptable.h 
	gcc -c communication.c -Wall -g
 
myrpfake.o: myrpfake.c myrpfake.h
	gcc -c myrpfake.c -Wall -g

cmirp.o: cmirp.c cmirp.h
	gcc -c cmirp.c -Wall -g

statistics.o: statistics.c statistics.h
	gcc -c statistics.c -Wall -g

simplelink.o: simplelink.c simplelink.h
	gcc -c simplelink.c -Wall -g

stabilizer.o: stabilizer.c stabilizer.h data.h
	gcc -c stabilizer.c -Wall -g

fake.o : fake.c fake.h data.h
	gcc -c fake.c -Wall -g

cal_inputs: cal_inputs.o cmirp.o data.o gwyfile.o
	gcc -o cal_inputs -pthread cal_inputs.o cmirp.o data.o gwyfile.o -lm  -g

cal_inputs.o: cal_inputs.c cmirp.h data.h gwyfile.h
	gcc -c cal_inputs.c -Wall -g

cal_outputs: cal_outputs.o cmirp.o data.o gwyfile.o
	gcc -o cal_outputs -pthread cal_outputs.o cmirp.o data.o gwyfile.o -lm -g

cal_outputs.o: cal_outputs.c cmirp.h data.h gwyfile.h
	gcc -c cal_outputs.c -Wall -g

cal_adc18b: cal_adc18b.o cmirp.o
	gcc -o cal_adc18b cal_adc18b.o cmirp.o -lm  -g

cal_adc18b.o: cal_adc18b.c cmirp.h
	gcc -c cal_adc18b.c -Wall -g

clean: 
	rm -f *.o hwserver hwserverfake test_client test_spi test_feedback test_kelvin test_nenoprobe test_lockin test_tapping test_akiyama confuseacat test_log_akiyama test_output test_dac cal_inputs cal_outputs cal_adc18b

dist-xz:
	rm -rf gwyhwserver
	mkdir gwyhwserver
	cp $(SOURCE_FILES) $(DATA_FILES) $(GNUPLOT_FILES) Makefile test_running.sh system_wrapper.bit AUTHORS TODO NEWS COPYING hwserver.service hwserver_cmitestsetup.ini gwyhwserver/
	tar -Jcvf gwyhwserver.tar.xz gwyhwserver/
	rm -rf gwyhwserver

dist-7z:
	rm -f gwyhwserver.tar.7z
	rm -rf gwyhwserver
	mkdir gwyhwserver
	cp $(SOURCE_FILES) $(DATA_FILES) $(GNUPLOT_FILES) Makefile test_running.sh system_wrapper.bit AUTHORS TODO NEWS COPYING hwserver.service hwserver_cmitestsetup.ini gwyhwserver/
	tar cf - gwyhwserver/ | 7zr a -si gwyhwserver.tar.7z
	rm -rf gwyhwserver

.PHONY: all clean dist-xz dist-7z
