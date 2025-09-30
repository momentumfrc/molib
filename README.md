# MoLib

This is a collection of utility classes developed over the past few years that we'd like to keep re-using in future
robot projects.

## Including in your robot project

1. Add this project as a git submodule

```bash
git submodule add git@github.com:momentumfrc/molib.git
```

2. Update your `build.gradle` to include files from the submodule by adding the following block:

```groovy
sourceSets {
    main {
        java {
            srcDirs = ['src/main/java', 'molib/src/main/java']
        }
    }
}
```

