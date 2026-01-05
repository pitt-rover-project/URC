# Contributing

This is a simple guide for contributing to Pitt's University Rover Challenge. If you have any questions, please contact either Nick Ferguson or Alexander Kwasinski. For an in-depth guide, use [MIT's missing semester](https://missing.csail.mit.edu/2020/version-control/).

1. The main branch is protected, you can not directly push commits to this branch. The main branch is considered the most recent production-ready code for the rover.

2. To perform work, please create a new branch to push commits into.
    * Please follow the branch convention of `users/{your username}/{feature}`
    * To enter a new branch: `git fetch` -> `git checkout <branch_name>`
    * To upload code should be: `git add .` -> `git commit -m "explain here"` -> `git push`
    * Other processes such as rebasing and merging may involve more complicated steps, so consult Nick or Alex.
        * To update your branch with recent changes made to the parent branch: `git rebase origin/main` (make sure you are in your branch)
        * Go through the pull request process to merge your branch with the main branch.

3. After completing and testing your code, make a pull request. You need at least one approval to merge any commits into the main branch.

4. After the merge, the branch must be deleted or archived.

This process ensure good quality code and that we do not have a pseudo main branch with code that is considered most recent and production-ready. 

## Commands

We would recommend using SSH to get git set up through GitHub. See this [guide](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/GitHub-SSH-Key-Setup-Config-Ubuntu-Linux) on how to do it. Then clone with SSH on this repository.

`git clone <url>` Clones a repository into your local development environment.  
`git branch` Check which you are on and see other branches (q to quit).  
`git fetch` Downloads commits, files, and refs from a remote repository into your local repository.  
`git pull` Fetch and download content from a remote repository and immediately update the local repository to match that content.  
`git checkout <branch>` Switches to a different branch.  
`git rebase <base>` [Git rebase](https://www.atlassian.com/git/tutorials/rewriting-history/git-rebase).  
`git add <files or .>` Saves changes to the staging area.  
`git commit -m "explain commit here"` Commits changes to the *LOCAL* repository.  
`git push` Pushes commits to the branch on GitHub.  

## Other tips

1. Keep commit descriptions accurate and clear. Do not put descriptions like '.' or 'u'.  
2. Commit often.  
3. If you encounter problems, ask!
