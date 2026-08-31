#!/bin/sh
# Amorce: delegue au hook versionne du depot.
#
# La copie precedente datait du 2026-08-29 et avait derive: les verifications
# ajoutees depuis ne tournaient pas, en silence. Un hook qu on copie a la main
# finit toujours perime — celui-ci ne contient donc aucune logique.
exec "$(git rev-parse --show-toplevel)/tools/hooks/pre-commit" "$@"
