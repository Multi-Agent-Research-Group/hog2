#------------------------------------------------------------------------------
# GNU Makefile: part for static libraries
#
# $Id: Makefile.lib.inc,v 1.1 2006/10/20 18:16:56 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile.lib.inc,v $
#------------------------------------------------------------------------------

$(DBG_BINDIR)/$(DBG_NAME): $(DBG_BINDIR)/$(DBG_SO_NAME)
	@echo "Creating $@"
	$(AR) -csr $@ $(DBG_OBJS)

$(DBG_BINDIR)/$(DBG_SO_NAME): $(DBG_OBJS)
	@echo "Creating $@"
	mkdir -p $(dir $@)
	$(CXX) -shared -o $@ $(DBG_OBJS)

$(REL_BINDIR)/$(REL_NAME): $(REL_OBJS)
	@echo "Creating $@"
	mkdir -p $(REL_BINDIR)
	$(AR) -csr $@ $(REL_OBJS)
