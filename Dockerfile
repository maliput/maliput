FROM gcr.io/cloud-builders/bazel

COPY . .

RUN bazel build //:all
RUN bazel test --test_output=all //:all