Li-san,
I am sending some sample codes for stream recording. Please try them when you have time. The code savepcd-separated-threads.cpp is most similar to your savepcd.cpp, so you can try this first. We can improve and polish these in future versions as necessary.
Regards,
Peter Davis

There were two issues to consider.

General issue:
The time required to write a point cloud (PC) to a file is not much less than the data arrival interval. So the handling has to be done efficiently or data will be missed.

Specific issue:
The method of transferring PC data between threads used in savepcd.cpp does not guarantee transferring every PC.

We provide three sample codes for you to test, in increasing order of expected reliability. (Note, the provided sample codes are based on savepcd.cpp and inherit some code that still needs to be cleaned and polished. This will be done in future updates.)

<savepcd-single-thread.cpp>
The file write is done in the VlpGrabber thread. This is simplest, but breaks down most easily if the write time is not fast enough.

<savepcd-separated-threads.cpp>
(most similar to savepcd)
The file write is done in the main thread using a mutex lock to hand the data from the VlpGrabber to the main thread. However, the use of mutex is done more reliably, avoiding the “try” method. This is more robust due to the concurrent operation of the two threads, but will still miss writing some data if the write operation is not fast enough.

<savepcd-separated-threads-with-queues.cpp>
The PC is enqueued in a FIFO queue in the VlpGrabber thread while the PCs in the queue are popped out and written to file in the main thread. This allows all PCs to be stored until it they can be written out, with record capacity limited only by the memory size.

Each code has the following additional features.

Under-sampling: Write only one frame every N frames via the writeEveryNFrames variable. Its value is hardcoded and must be tuned before the compilation (default is writeEveryNFrames = 1).

List of recorded data: Separate write to standard-output can be used to log the frame number and receive time (in 𝜇s) of each point cloud written.


这三个Julien都在VM上试过，都能存储全部数据。

保存为pcap:
文件系统对文件大小有限制，估计3G左右，而且太大了也无法在机器上打开处理，因为内存不够。
但tshark可以在记录时自动分割成多个pcap文件。所以这个方法是可行的，只是直接作为数据包保存可能会有更多冗余。
