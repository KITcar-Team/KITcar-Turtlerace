# KITcar turtle race
This repository is part of the KITcar onboarding.
You find the instructions for what to do with the repo in the wiki.

# Prerequisite
In order to complete the turtlerace-onboarding,
you need to have your own [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo)
of the repository.
For that, viewing this repo in your browser on GitHub, click on the "fork"-button in the top right
and follow the wizard.
Don't change the suggested name.
This will create the forked repository at `https://github.com/<your username>/KITcar-Turtlerace`.

# Setup
We recommend to work with the repository in combination with [kitcar-ws](https://github.com/KITcar-Team/kitcar-ws).

Here are the instructions on how to achieve this setup.

1.  Run `git clone https://github.com/KITcar-Team/kitcar-ws.git` (it
    will clone the repository into the folder where you run it in. A
    common place would be for example in a folder like `~/kitcar/`. If
    you don’t know what that means, read about it
    [here](<https://linuxvox.com/blog/home-in-linux/>) or ask your
    favorite LLM).

2.  After that, you need to navigate into the `src` folder of the
    workspace-repo. To do that, just run `cd kitcar-ws/src`.

3.  Now, you need to clone the repository you had forked earlier. Run
    `git clone https://github.com/<your username>/KITcar-Turtlerace`.

4.  If that worked, you have all you need on disk. Now open vscode and
    go to `File>Open Folder`.

5.  In the file explorer, select the `kitcar-ws` folder from before.
    **Do not select the turtlerace folder! You need to navigate to the
    `kitcar-ws` folder and click on *Open Folder*!**

6.  Now, vscode should suggest you to re-open the workspace in the
    devcontainer. Either click on the blue button of the dialog or open
    the command palette (CTRL+SHIFT+P) and search for *Open in
    Dev Containers: Reopen in Container*. Now wait until vscode is
    ready again and pray that everything works.

Here is a summary of the commands that you need to run
(remember to first fork the repository, and then replace `<your username>` with your actual username below):
```bash
git clone https://github.com/KITcar-Team/kitcar-ws.git
cd kitcar-ws/src
git clone https://github.com/<your username>/KITcar-Turtlerace
```
