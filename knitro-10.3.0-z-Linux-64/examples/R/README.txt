====================================================
===       Important notes on R interface         ===
====================================================

Release: v1.2 for Knitro 10.3

----------------
IMPORTANT NOTICE
----------------
This is the R interface for Artelys Knitro.

Please note that because of some limitations from R some platform
specific issues may arise. Please contact Knitro support team
in this case to get support.

For more information on R interface, please refer to Knitro user guide.

Comments and feedbacks on this interface are welcome and should
be addressed to support-knitro@artelys.com.

---------------------------------------
HOW TO INSTALL AND USE THE R INTERFACE
---------------------------------------

R interface can be installed by running the following command
from the R command prompt:
  > install.packages("KR_PACKAGE", repos=NULL)

where "KR_PACKAGE" should be replaced by KnitroR compressed package
name:
  - "KnitroR_1.2.0.zip" for windows,
  - "KnitroR_1.2.0_x86_64-pc-linux-gnu.tar.gz"  for Linux,
  - "KnitroR_1.2.0.tgz" for Mac OS X.

Very important: In order to run the KnitroR functions, you need to first have the KNITRODIR environment defined
and pointing to your Knitro local installation directory, where you should have the Knitro library
in the lib/ subdirectory.

Then, Knitro can be used directly from R as follows:
  > knitro(objective=function(x) x[1]*x[2], xL=c(-5,-5), xU=c(10,10))

And examples can be run by sourcing the examples files:
  > source("examples/HS71.R")

