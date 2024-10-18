# Contributing

Hi there!
We’re thrilled that you’d like to contribute to this project. We aim to make contributing to Qualcomm QRB ROS is easy, enjoyable, and educational for everyone. All contributions are welcome, whether they are features, issues, documentation, guides, or anything else. Your help is essential for keeping this project great and for making it better.

## Contributing Guidelines
Before you submit a pull request to QRB ROS project, please ensure that you have followed these guidelines:
- Code should be well-documented and formatted according to the [Code style and language versions](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- All changes should be covered by tests.
- Commits should be well-written and descriptive, with a clear summary of the changes made and any relevant context.
- Pull requests should target the `main` branch and include a clear summary of the changes made.

## Bug report
If you find a bug in the source code, you can help us by [submitting an issue](../../issues). Even better, you can submit a [Pull Request](../../pulls) with a fix.

## New feature
You can request a new feature by [submitting an issue](../../issues).
<Update link with template>

If you'd like to implement a new feature, it's always good to be in touch with us before you invest time and effort, since not all features can be supported.

- For a Major Feature, first open an issue and outline your proposal. This will let us coordinate efforts, prevent duplication of work, and help you craft the change so that it's successfully integrated into the project.
- Small Features can be crafted and directly [submitted as a Pull Request](#submitting-a-pull-request).


# How to make a code contribution

## Open to community
You can check all the issues that are open for community contributions. Check [issues](../../issues).

## Submitting a pull request

Please read our [code of conduct](CODE-OF-CONDUCT.md) and [license](LICENSE).

### Create a fork
[Fork](../../fork) the repository to your GitHub organization. This means that you'll have a copy of the repository under your-username/repository-name.

1. Go to the repository page.
2. Click the "Fork" button in the top-right corner of the page.
3. Follow the instructions to create a fork of the repository in your own GitHub account.

### Clone the forked repository

```bash
git clone https://github.com/<your-username>/<repository-name>
```
Replace your-username and repository-name with your GitHub username and the name of the forked repository.

### Create a new branch
Create a new branch based on `main` for your changes.

```bash
git checkout -b <my-branch-name> main
```

### Make your changes
Make your changes with bug fix or new feature.

### Commit your changes
Use the [DCO](http://developercertificate.org/). You can attest to the DCO by commiting with the **-s** or **--signoff** options or manually adding the "Signed-off-by":
```bash
git add .
git commit -s -m "<type>: <subject>"
```
<add commit template>

### Push your changes
```bash
git push -u origin <my-branch-name>
```
    The `-u` is shorthand for `--set-upstream`. This will set up the tracking reference so subsequent runs of `git push` or `git pull` can omit the remote and branch.

### Create pull request
- Go to your forked repository on GitHub.
- You should see a prompt to compare and create a pull request for your recently pushed branch. Click on it.
- If you don’t see the prompt, go to the “Pull requests” tab and click the “New pull request” button.
- Select your branch from the “compare” dropdown.
- Add a title and description for your pull request, explaining what changes you made and why.
- Click “Create pull request”.
- Wait for the pull request to be reviewed by a maintainer.
- Make changes to the pull request if the reviewing maintainer recommends them.

Celebrate your success after your pull request is merged.


Here are a few things you can do that will increase the likelihood of your pull request to be accepted:

- Follow the existing style where possible. **[Code style and language versions](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)**
- Write tests.
- Keep your change as focused as possible.
  If you want to make multiple independent changes, please consider submitting them as separate pull requests.
- Write a [good commit message](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html).
- It's a good idea to arrange a discussion with other developers to ensure there is consensus on large features, architecture changes, and other core code changes. PR reviews will go much faster when there are no surprises.
