#include <array>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include <time.h>
#include <vector>

using namespace std;

const uint8_t POS_COUNT = 24, PIECE_COUNT = 9;

enum class Stage : uint8_t { DROP, MOVE, JUMP, BLACKS_WON, WHITES_WON };

enum class Cell : uint8_t { EMPTY = 0, WHITES, BLACKS };

inline char cellRepr(Cell c) {
  switch (c) {
  case Cell::EMPTY:
    return '+';
  case Cell::WHITES:
    return 'W';
  case Cell::BLACKS:
    return 'B';
  }
}

enum class Player : uint8_t { WHITES, BLACKS };

enum class Failure : uint8_t {
  GAME_OVER,
  SHOULD_DROP,
  CANNOT_DROP,
  CANNOT_JUMP,
  DEST_IS_TAKEN,
  SOURCE_IS_EMPTY,
  SOURCE_IS_OPPONENT,
  MUST_EAT,
  MUST_EAT_FROM_OPPONENT,
  CANNOT_EAT_FROM_MILL,
  NONE
};

const std::string messageForFailure(Failure f) {
  switch (f) {
  case Failure::GAME_OVER:
    return "game is already over";
  case Failure::SHOULD_DROP:
    return "should be a drop";
  case Failure::CANNOT_DROP:
    return "drop not allowed";
  case Failure::CANNOT_JUMP:
    return "jump not allowed";
  case Failure::DEST_IS_TAKEN:
    return "destination is already taken";
  case Failure::SOURCE_IS_EMPTY:
    return "no piece there";
  case Failure::SOURCE_IS_OPPONENT:
    return "not your piece";
  case Failure::MUST_EAT:
    return "must eat";
  case Failure::MUST_EAT_FROM_OPPONENT:
    return "must eat a piece from your opponent";
  case Failure::CANNOT_EAT_FROM_MILL:
    return "cannot eat from a mill when the opponent has pieces outside of a "
           "mill";
  case Failure::NONE:
    return "no failure";
  }
}

enum class ActionType : uint8_t { DROP, MOVE, JUMP };

enum class Pos : int8_t {
  A = 0,
  B = 1,
  C = 2,
  D = 3,
  E = 4,
  F = 5,
  G = 6,
  H = 7,
  I = 8,
  J = 9,
  K = 10,
  L = 11,
  M = 12,
  N = 13,
  O = 14,
  P = 15,
  Q = 16,
  R = 17,
  S = 18,
  T = 19,
  U = 20,
  V = 21,
  W = 22,
  X = 23,
  NONE
};

constexpr Cell cellForPlayer(Player player) {
  return player == Player::WHITES ? Cell::WHITES : Cell::BLACKS;
}

constexpr Cell cellForOtherPlayer(Player player) {
  return player == Player::WHITES ? Cell::BLACKS : Cell::WHITES;
}

using Line = std::array<Pos, 3>;
using PeersInLine = std::array<Pos, 2>;
using Peers = std::array<PeersInLine, 2>;
using Neighbors = std::vector<Pos>;
template <typename E> using PosArray = std::array<E, POS_COUNT>;
using Board = std::array<Cell, POS_COUNT>;

const PosArray<Line> LINES = {{{Pos::A, Pos::B, Pos::C},
                               {Pos::D, Pos::E, Pos::F},
                               {Pos::G, Pos::H, Pos::I},
                               {Pos::J, Pos::K, Pos::L},
                               {Pos::M, Pos::N, Pos::O},
                               {Pos::P, Pos::Q, Pos::R},
                               {Pos::S, Pos::T, Pos::U},
                               {Pos::V, Pos::W, Pos::X},
                               {Pos::A, Pos::J, Pos::V},
                               {Pos::D, Pos::K, Pos::S},
                               {Pos::G, Pos::L, Pos::P},
                               {Pos::B, Pos::E, Pos::H},
                               {Pos::Q, Pos::T, Pos::W},
                               {Pos::I, Pos::M, Pos::R},
                               {Pos::F, Pos::N, Pos::U},
                               {Pos::C, Pos::O, Pos::X}}};

const PosArray<Peers> PEERS = {{
    {{{{Pos::B, Pos::C}}, {{Pos::J, Pos::V}}}},
    {{{{Pos::A, Pos::C}}, {{Pos::E, Pos::H}}}},
    {{{{Pos::A, Pos::B}}, {{Pos::O, Pos::X}}}},
    {{{{Pos::E, Pos::F}}, {{Pos::K, Pos::S}}}},
    {{{{Pos::D, Pos::F}}, {{Pos::B, Pos::H}}}},
    {{{{Pos::D, Pos::E}}, {{Pos::N, Pos::U}}}},
    {{{{Pos::H, Pos::I}}, {{Pos::L, Pos::P}}}},
    {{{{Pos::G, Pos::I}}, {{Pos::B, Pos::E}}}},
    {{{{Pos::G, Pos::H}}, {{Pos::M, Pos::R}}}},
    {{{{Pos::K, Pos::L}}, {{Pos::A, Pos::V}}}},
    {{{{Pos::J, Pos::L}}, {{Pos::D, Pos::S}}}},
    {{{{Pos::J, Pos::K}}, {{Pos::G, Pos::P}}}},
    {{{{Pos::N, Pos::O}}, {{Pos::I, Pos::R}}}},
    {{{{Pos::M, Pos::O}}, {{Pos::F, Pos::U}}}},
    {{{{Pos::M, Pos::N}}, {{Pos::C, Pos::X}}}},
    {{{{Pos::Q, Pos::R}}, {{Pos::G, Pos::L}}}},
    {{{{Pos::P, Pos::R}}, {{Pos::T, Pos::W}}}},
    {{{{Pos::P, Pos::Q}}, {{Pos::I, Pos::M}}}},
    {{{{Pos::T, Pos::U}}, {{Pos::D, Pos::K}}}},
    {{{{Pos::S, Pos::U}}, {{Pos::Q, Pos::W}}}},
    {{{{Pos::S, Pos::T}}, {{Pos::F, Pos::N}}}},
    {{{{Pos::W, Pos::X}}, {{Pos::A, Pos::J}}}},
    {{{{Pos::V, Pos::X}}, {{Pos::Q, Pos::T}}}},
    {{{{Pos::V, Pos::W}}, {{Pos::C, Pos::O}}}},
}};

const PosArray<Neighbors> NEIGHBORS = {{{Pos::B, Pos::J},
                                        {Pos::A, Pos::C},
                                        {Pos::B, Pos::O},
                                        {Pos::E, Pos::K},
                                        {Pos::B, Pos::D, Pos::F, Pos::H},
                                        {Pos::E, Pos::N},
                                        {Pos::H, Pos::L},
                                        {Pos::E, Pos::G, Pos::I},
                                        {Pos::H, Pos::M},
                                        {Pos::A, Pos::K, Pos::V},
                                        {Pos::D, Pos::J, Pos::L, Pos::S},
                                        {Pos::G, Pos::K, Pos::L},
                                        {Pos::I, Pos::N, Pos::R},
                                        {Pos::F, Pos::M, Pos::O, Pos::U},
                                        {Pos::C, Pos::N, Pos::X},
                                        {Pos::L, Pos::Q},
                                        {Pos::P, Pos::R, Pos::T},
                                        {Pos::M, Pos::Q},
                                        {Pos::K, Pos::T},
                                        {Pos::Q, Pos::S, Pos::U, Pos::W},
                                        {Pos::N, Pos::T},
                                        {Pos::J, Pos::W},
                                        {Pos::T, Pos::V, Pos::X},
                                        {Pos::O, Pos::W}}};

constexpr char posChar(Pos p) { return 'A' + static_cast<char>(p); }

constexpr Pos charPos(char c) { return static_cast<Pos>(c - 'A'); }

const ActionType determineActionType(Pos to, Pos from) {
  if (from == Pos::NONE) {
    return ActionType::DROP;
  } else {
    const auto neighbors = NEIGHBORS[static_cast<size_t>(to)];
    for (const auto neigh : neighbors) {
      if (neigh == from) {
        return ActionType::MOVE;
      }
    }
    return ActionType::JUMP;
  }
}

class Action {
  const Pos to_;
  const Pos from_;
  const Pos eats_;
  const ActionType type_;

public:
  Action(Pos to, Pos from, Pos eats)
      : to_(to), from_(from), eats_(eats),
        type_(determineActionType(to, from)){};

  const Pos from() const { return from_; }

  const Pos to() const { return to_; }

  const Pos eats() const { return eats_; }

  const ActionType type() const { return type_; }

  const std::string toString() const {
    std::string res;
    if (from_ != Pos::NONE) {
      res += posChar(from_);
    }
    res += posChar(to_);
    if (eats_ != Pos::NONE) {
      res += posChar(eats_);
    }
    return res;
  }
};

class Game {
  const Player player_;
  const Stage stage_;
  const std::vector<Action> actions_;
  const Board board_;
  const Board mills_;
  const Failure failure_;

private:
  inline bool canEat(const Action action) const;

  inline const std::vector<Action> withEat(const Action action) const;

  void pushAllFor(const Action action, std::vector<Action> *dest) const;

  Game(const Player player, const Stage stage,
       const std::vector<Action> actions, const Board board, const Board mills,
       const Failure failure);

public:
  Game();

  const std::string toString() const;

  Game after(const Action action, const bool validate) const;

  Failure failureForAction(const Action action) const;

  std::vector<Action> possibleActions() const;

  void finishRandomly(bool log) const;

  inline bool finished() const;
};

Game::Game()
    : player_(Player::WHITES), stage_(Stage::DROP), actions_({}),
      failure_(Failure::NONE), board_({Cell::EMPTY}), mills_({Cell::EMPTY}) {}

Game::Game(Player player, Stage stage, std::vector<Action> actions, Board board,
           Board mills, Failure failure)
    : player_(player), stage_(stage), actions_(actions), board_(board),
      mills_(mills), failure_(failure) {}

Game Game::after(const Action action, const bool validate) const {
  const auto player = player_;
  const auto nextPlayer =
      player_ == Player::WHITES ? Player::BLACKS : Player::WHITES;
  const auto ourCell = cellForPlayer(player),
             theirCell = cellForOtherPlayer(player);

  auto nextBoard = Board(board_);
  auto nextFailure = failure_;

  auto nextActions = std::vector<Action>(actions_);
  nextActions.push_back(action);

  const auto from = action.from(), to = action.to(), eats = action.eats();
  const auto fromi = static_cast<size_t>(from), toi = static_cast<size_t>(to),
             eatsi = static_cast<size_t>(eats);

  auto nextStage = Stage::DROP;

  if (nextFailure == Failure::NONE && validate) {
    nextFailure = failureForAction(action);
  }

  if (nextFailure == Failure::NONE) {
    nextBoard[toi] = ourCell;
    if (from != Pos::NONE) {
      nextBoard[fromi] = Cell::EMPTY;
    }
    if (eats != Pos::NONE) {
      nextBoard[eatsi] = Cell::EMPTY;
    }

    if (nextActions.size() > PIECE_COUNT * 2) {
      // We're not dropping anymore
      uint8_t whitesCount = 0, blacksCount = 0;
      bool whitesCanMove = false, blacksCanMove = false;

      for (auto pos = 0; pos < POS_COUNT; ++pos) {
        const auto cell = nextBoard[pos];
        switch (cell) {
        case Cell::BLACKS:
          ++blacksCount;
          if (!blacksCanMove) {
            const auto neighs = NEIGHBORS[pos];
            for (const auto neigh : neighs) {
              if (nextBoard[static_cast<size_t>(neigh)] == Cell::EMPTY) {
                blacksCanMove = true;
                break;
              }
            }
          }
          break;
        case Cell::WHITES:
          ++whitesCount;
          if (!whitesCanMove) {
            const auto neighs = NEIGHBORS[pos];
            for (const auto neigh : neighs) {
              if (nextBoard[static_cast<size_t>(neigh)] == Cell::EMPTY) {
                whitesCanMove = true;
                break;
              }
            }
          }
          break;
        }
      }

      if (whitesCount == 3) {
        whitesCanMove = true;
      }
      if (blacksCount == 3) {
        blacksCanMove = true;
      }

      if (whitesCount <= 2 || (!whitesCanMove && whitesCount != 3)) {
        nextStage = Stage::BLACKS_WON;
      } else if (blacksCount <= 2 || (!blacksCanMove && blacksCount != 3)) {
        nextStage = Stage::WHITES_WON;
      } else {
        if ((nextPlayer == Player::WHITES && whitesCount == 3) ||
            (nextPlayer == Player::BLACKS && blacksCount == 3)) {
          nextStage = Stage::JUMP;
        } else {
          nextStage = Stage::MOVE;
        }
      }
    }
  }

  auto nextMills = Board();
  for (const auto line : LINES) {
    const auto a = static_cast<size_t>(line[0]),
               b = static_cast<size_t>(line[1]),
               c = static_cast<size_t>(line[2]);
    const auto cell = nextBoard[a];
    if (nextBoard[b] == cell && nextBoard[c] == cell) {
      nextMills[a] = nextMills[b] = nextMills[c] = cell;
    }
  }

  return Game(nextPlayer, nextStage, nextActions, nextBoard, nextMills,
              nextFailure);
}

inline bool Game::canEat(const Action action) const {
  const auto neededInLine = cellForPlayer(player_);
  const auto peers = PEERS[static_cast<size_t>(action.to())];
  const auto from = action.from();
  const auto aa = peers[0][0], ab = peers[0][1], ba = peers[1][0],
             bb = peers[1][1];
  return ((aa != from && ab != from &&
           board_[static_cast<size_t>(aa)] == neededInLine &&
           board_[static_cast<size_t>(ab)] == neededInLine) ||
          (ba != from && bb != from &&
           board_[static_cast<size_t>(ba)] == neededInLine &&
           board_[static_cast<size_t>(bb)] == neededInLine));
}

const std::string Game::toString() const {
  std::string res;
  switch (stage_) {
  case Stage::WHITES_WON:
    res += "Whites won";
    break;
  case Stage::BLACKS_WON:
    res += "Blacks won";
    break;
  case Stage::DROP:
  case Stage::MOVE:
  case Stage::JUMP:
    if (failure_ == Failure::NONE) {
      res += player_ == Player::WHITES ? "Whites play" : "Blacks play";
    } else {
      res += messageForFailure(failure_);
    }
  }
  res += "\nActions: ";
  for (const auto action : actions_) {
    res += action.toString();
    res += ',';
  }
  res.pop_back();
  res += '\n';
  res += cellRepr(board_[0]);
  res += "-----";
  res += cellRepr(board_[1]);
  res += "-----";
  res += cellRepr(board_[2]);
  res += "\n| ";
  res += cellRepr(board_[3]);
  res += "---";
  res += cellRepr(board_[4]);
  res += "---";
  res += cellRepr(board_[5]);
  res += " |\n| | ";
  res += cellRepr(board_[6]);
  res += '-';
  res += cellRepr(board_[7]);
  res += '-';
  res += cellRepr(board_[8]);
  res += " | |\n";
  res += cellRepr(board_[9]);
  res += '-';
  res += cellRepr(board_[10]);
  res += '-';
  res += cellRepr(board_[11]);
  res += "   ";
  res += cellRepr(board_[12]);
  res += '-';
  res += cellRepr(board_[13]);
  res += '-';
  res += cellRepr(board_[14]);
  res += "\n| | ";
  res += cellRepr(board_[15]);
  res += '-';
  res += cellRepr(board_[16]);
  res += '-';
  res += cellRepr(board_[17]);
  res += " | |\n| ";
  res += cellRepr(board_[18]);
  res += "---";
  res += cellRepr(board_[19]);
  res += "---";
  res += cellRepr(board_[20]);
  res += " |\n";
  res += cellRepr(board_[21]);
  res += "-----";
  res += cellRepr(board_[22]);
  res += "-----";
  res += cellRepr(board_[23]);
  return res;
}

Failure Game::failureForAction(const Action action) const {
  const auto type = action.type();
  const auto from = action.from(), to = action.to(), eats = action.eats();
  const auto fromi = static_cast<size_t>(from), toi = static_cast<size_t>(to),
             eatsi = static_cast<size_t>(eats);
  const auto opponentCell = cellForOtherPlayer(player_);

  if (finished())
    return Failure::GAME_OVER;

  if (stage_ == Stage::DROP) {
    if (type != ActionType::DROP) {
      return Failure::SHOULD_DROP;
    }
  } else {
    if (type == ActionType::DROP) {
      return Failure::CANNOT_DROP;
    } else if (type == ActionType::JUMP && stage_ != Stage::MOVE) {
      return Failure::CANNOT_JUMP;
    }
  }

  if (board_[toi] != Cell::EMPTY) {
    return Failure::DEST_IS_TAKEN;
  }

  if (from != Pos::NONE) {
    const auto fromCell = board_[fromi];
    if (fromCell == Cell::EMPTY) {
      return Failure::SOURCE_IS_EMPTY;
    } else if (fromCell == opponentCell) {
      return Failure::SOURCE_IS_OPPONENT;
    }
  }

  if (canEat(action)) {
    if (eats == Pos::NONE) {
      return Failure::MUST_EAT;
    }
    if (board_[eatsi] != opponentCell) {
      return Failure::MUST_EAT_FROM_OPPONENT;
    }
    if (mills_[eatsi] != Cell::EMPTY) {
      for (auto i = 0; i < POS_COUNT; i++) {
        if (board_[i] == opponentCell && mills_[i] == Cell::EMPTY) {
          return Failure::CANNOT_EAT_FROM_MILL;
        }
      }
    }
  }

  return Failure::NONE;
};

inline const std::vector<Action> Game::withEat(const Action action) const {
  auto withMills = std::vector<Action>(), withoutMills = std::vector<Action>();
  withMills.reserve(PIECE_COUNT);
  withoutMills.reserve(PIECE_COUNT);
  const auto eatable = cellForOtherPlayer(player_);
  for (auto pos = 0; pos < POS_COUNT; ++pos) {
    if (board_[pos] == eatable) {
      const auto eat =
          Action(action.to(), action.from(), static_cast<Pos>(pos));
      withMills.push_back(eat);
      if (mills_[pos] == Cell::EMPTY) {
        withoutMills.push_back(eat);
      }
    }
    return withoutMills.empty() ? withMills : withoutMills;
  }
}

void Game::pushAllFor(const Action action, std::vector<Action> *dest) const {
  if (canEat(action)) {
    const auto actions = withEat(action);
    dest->reserve(dest->size() + actions.size());
    for (const auto act : actions) {
      dest->push_back(act);
    }
  } else {
    dest->push_back(action);
  }
}

std::vector<Action> Game::possibleActions() const {
  auto res = std::vector<Action>();
  const auto ours = cellForPlayer(player_);

  switch (stage_) {
  case Stage::BLACKS_WON:
  case Stage::WHITES_WON:
    break;
  case Stage::DROP:
    for (auto pos = 0; pos < POS_COUNT; ++pos) {
      if (board_[pos] == Cell::EMPTY) {
        pushAllFor(Action(static_cast<Pos>(pos), Pos::NONE, Pos::NONE), &res);
      }
    }
    break;
  case Stage::MOVE:
    for (auto from = 0; from < POS_COUNT; ++from) {
      if (board_[from] == ours) {
        const auto neighbors = NEIGHBORS[from];
        for (const auto to : neighbors) {
          if (board_[static_cast<size_t>(to)] == Cell::EMPTY) {
            pushAllFor(Action(to, static_cast<Pos>(from), Pos::NONE), &res);
          }
        }
      }
    }
    break;
  case Stage::JUMP:
    for (auto from = 0; from < POS_COUNT; ++from) {
      if (board_[from] == ours) {
        for (auto to = 0; to < POS_COUNT; ++to) {
          if (board_[to] == Cell::EMPTY) {
            pushAllFor(
                Action(static_cast<Pos>(to), static_cast<Pos>(from), Pos::NONE),
                &res);
          }
        }
      }
    }
    break;
  }
  return res;
}

void Game::finishRandomly(bool log) const {
  const Game *game = new Game(*this);
  if (log)
    std::cout << game->toString() << std::endl;
  while (!game->finished()) {
    const auto actions = game->possibleActions();
    const Action action = actions[rand() % actions.size()];
    const Game *tmp = game;
    game = new Game(game->after(action, false));
    if (tmp != this)
      delete tmp;
    if (log)
      std::cout << game->toString() << std::endl;
  }
  delete game;
}

bool Game::finished() const {
  return stage_ == Stage::BLACKS_WON || stage_ == Stage::WHITES_WON;
}

int main() {
  srand(time(NULL));
  for (auto i = 0; i < 1000; ++i) {
    Game().finishRandomly(false);
  }
  return 0;
}
