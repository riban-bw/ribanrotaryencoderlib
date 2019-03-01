CXX = g++
LIBS = -lpthread
src = $(wildcard *.cpp)
obj = $(src:.cpp=.o)
dep = $(obj:.o=.d)

enctest: $(obj)
	$(CXX) -o $@ $(LIBS) $^

-include $(dep)

# rule to generate dependency files using C preprocessor
%.d: %.cpp
	@$(CPP) $(CFLAGS) $< -MM -MT $(@:.d=.o) >$@

.PHONY: clean
clean:
	rm -r $(obj) enctest $(dep)
