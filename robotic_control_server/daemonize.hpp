//
// Created by sushant on 8/7/19.
//

#ifndef ROBOTIC_CONTROL_SERVER_DAEMONIZE_HPP
#define ROBOTIC_CONTROL_SERVER_DAEMONIZE_HPP

#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

static void daemonize() {
  pid_t pid;

  /* Fork off the parent process */
  pid = fork();

  /* An error occurred */
  if (pid < 0)
    exit(EXIT_FAILURE);

  /* Success: Let the parent terminate */
  if (pid > 0)
    exit(EXIT_SUCCESS);

  /* On success: The child process becomes session leader */
  if (setsid() < 0)
    exit(EXIT_FAILURE);

  /* Catch, ignore and handle signals */
  // TODO: Implement a working signal handler */
  signal(SIGCHLD, SIG_IGN);
  signal(SIGHUP, SIG_IGN);

  /* Fork off for the second time*/
  pid = fork();

  /* An error occurred */
  if (pid < 0)
    exit(EXIT_FAILURE);

  /* Success: Let the parent terminate */
  if (pid > 0)
    exit(EXIT_SUCCESS);

  /* Set new file permissions */
  umask(0);

  /* Change the working directory to the root directory */
  /* or another appropriated directory */
  chdir("/home/pi");

  /* Close all open file descriptors */
  int x;
  for (x = sysconf(_SC_OPEN_MAX); x >= 0; x--) {
    close(x);
  }

  // reopen stdin, stdout, stderr
  stdin = fopen(NULL, "r");                                    // fd=0
  stdout = fopen("/var/log/robotic_control_server.log", "w+"); // fd=1
  stderr = fopen(NULL, "w+");                                  // fd=2

  pid_t to_write_pid = getpid();
  std::ofstream outfile("/var/run/robotic_control_server.pid",
                        std::ofstream::out);
  outfile << to_write_pid;
  outfile.close();

  /* Open the log file */
  //    openlog ("firstdaemon", LOG_PID, LOG_DAEMON);
}

#endif // ROBOTIC_CONTROL_SERVER_DAEMONIZE_HPP
